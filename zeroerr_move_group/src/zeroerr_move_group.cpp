#include <zeroerr_move_group/zeroerr_move_group.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)

ArmMoveGroup::ArmMoveGroup()
{
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	mg_node_ = rclcpp::Node::make_shared("mg_node_", node_options);
	node_ = rclcpp::Node::make_shared(NODE_NAME, node_options);

	using namespace std::placeholders;

	collision_obj_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/SetupLabEnvironment",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::coll_obj_cb_, this, _1)
	);

	pose_array_sub_ = node_->create_subscription<zeroerr_msgs::msg::PoseTargetArray>(
		"arm/PoseGoalArray",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::pose_array_cb_, this, _1)
	);

	pose_sub_ = node_->create_subscription<zeroerr_msgs::msg::PoseTarget>(
		"arm/PoseGoal",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::pose_cb_, this, _1)
	);

	arm_execute_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Execute",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::execute_cb_, this, _1)
	);

	arm_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Stop",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::stop_cb_, this, _1)
	);

	arm_clear_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Clear",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::clear_cb_, this, _1)
	);


	joint_space_goal_srv_ = node_->create_service<JointSpaceGoal>(
		"arm/JointSpaceGoal",
		std::bind(&ArmMoveGroup::joint_space_goal_cb_, this, _1, _2)
	);

	save_srv_ = node_->create_service<Save>(
		"arm/Save", 
		std::bind(&ArmMoveGroup::save_cb_, this, _1, _2)
	);

	move_to_saved_srv_ = node_->create_service<MoveToSaved>(
		"arm/ExecuteSaved",
		std::bind(&ArmMoveGroup::execute_saved_cb_, this, _1, _2)
	);


	RCLCPP_INFO(node_->get_logger(), "Initialized!");
}

ArmMoveGroup::~ArmMoveGroup()
{
	RCLCPP_INFO(node_->get_logger(), "Destruct sequence initiated.");

	table_.operation = table_.REMOVE;
	planning_scene_interface_.applyCollisionObject(table_);
}



// void ArmMoveGroup::coll_obj_cb_(zeroerr_msgs::msg::CollisionObject::SharedPtr coll_obj_msg)
void ArmMoveGroup::coll_obj_cb_(std_msgs::msg::Bool::SharedPtr coll_obj_msg)
{
	RCLCPP_INFO(node_->get_logger(), "Adding collision object!");

	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	//* Place table underneath arm
	table_.header.frame_id = move_group.getPlanningFrame();

	table_.id = "table1";

	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.91;
	primitive.dimensions[primitive.BOX_Y] = 1.54;
	primitive.dimensions[primitive.BOX_Z] = 0.08;

	geometry_msgs::msg::Pose table_pose;
	table_pose.orientation.w = 1.0;
	table_pose.position.x = 0.405;
	table_pose.position.y = -0.67;
	table_pose.position.z = -0.07;

	table_.primitives.push_back(primitive);
	table_.primitive_poses.push_back(table_pose);
	table_.operation = table_.ADD;

	planning_scene_interface_.applyCollisionObject(table_);
}


void ArmMoveGroup::joint_space_goal_cb_(const std::shared_ptr<JointSpaceGoal::Request> request, std::shared_ptr<JointSpaceGoal::Response> response)
{
	RCLCPP_INFO(node_->get_logger(), "Joint space goal received.");
	

	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	move_group.startStateMonitor();

	// Use STOMP
	move_group.setPlanningPipelineId("stomp");

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	move_group.setStartState(*current_state);

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (uint i = 0; i < NUM_JOINTS; i++)
		joint_group_positions[i] = ((request->joint_pos_deg[i] * PI) / 180); // Deg -> Rad
	// RCLCPP_INFO(node_->get_logger(), "Converted deg to rad!\n");


	float vel_scaling_factor = (float) (request->speed / 100.0);
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);
	// RCLCPP_INFO(node_->get_logger(), "Velocity scaling factor: %f(%u/100)", vel_scaling_factor, goal_msg->speed);


	bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
	if (!within_bounds)
	{
		RCLCPP_WARN(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
	}
	// RCLCPP_INFO(node_->get_logger(), "Motion plan within bounds!\n");

	bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
	{
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
		response->valid = true;
	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
		response->valid = false;
	}
}


void ArmMoveGroup::pose_array_cb_(zeroerr_msgs::msg::PoseTargetArray::SharedPtr pose_array_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	move_group.startStateMonitor(2.0);
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	moveit::core::RobotState start_state(*move_group.getCurrentState());
	move_group.setStartState(start_state);

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// for (uint i = 0; i < NUM_JOINTS; i++)
	// 	RCLCPP_INFO(node_->get_logger(), "Current J%d angle: %f", i + 1, joint_group_positions[i]);

	const std::string type = pose_array_msg->type;
	if (!strcmp(type.c_str(), "linear"))
	{
		linear_trajectory_recv_ = true;

		std::vector<geometry_msgs::msg::Pose> waypoints;
		// waypoints.push_back(move_group.getCurrentPose().pose);

		RCLCPP_INFO(node_->get_logger(), "Pose array receieved with %lu waypoints.", pose_array_msg->waypoints.size());
		// move_group.setMaxVelocityScalingFactor(0.1);

		for (size_t i = 0; i < pose_array_msg->waypoints.size(); i++)
			waypoints.push_back(pose_array_msg->waypoints[i]);
		
		// moveit_msgs::msg::RobotTrajectory trajectory;
		// const double jump_threshold = 0.0;	// Disable jump threshold
		// const double eef_step = 0.01; 		// 1cm interpolation resolution
		const double jump_threshold = pose_array_msg->jump_threshold;
		const double eef_step = pose_array_msg->step_size;
		double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_);

		RCLCPP_INFO(node_->get_logger(), "Planning cartesian path (%.2f%% achieved)", fraction * 100.0);
	}
	else if(!strcmp(type.c_str(), "arc"))
	{
		move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
		move_group.setPlannerId("CIRC");


		geometry_msgs::msg::PoseStamped center;
		geometry_msgs::msg::PoseStamped endpoint;

		center.pose = pose_array_msg->waypoints[0];
		center.header.frame_id = "base_link";
		endpoint.pose = pose_array_msg->waypoints[1];
		endpoint.header.frame_id = "base_link";


		move_group.setPoseTarget(endpoint);
		

		moveit_msgs::msg::Constraints constraints;
		moveit_msgs::msg::PositionConstraint pos_constraint;
		constraints.name = "center";
		pos_constraint.header.frame_id = center.header.frame_id;
		pos_constraint.link_name = "j6_Link";
		pos_constraint.constraint_region.primitive_poses.push_back(center.pose);
		pos_constraint.weight = 1.0;
		constraints.position_constraints.push_back(pos_constraint);
		move_group.setPathConstraints(constraints);

		bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success)
			RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
		else
			RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");

		move_group.clearPathConstraints();
	}

}


void ArmMoveGroup::pose_cb_(zeroerr_msgs::msg::PoseTarget::SharedPtr goal_msg)
{
	RCLCPP_INFO(node_->get_logger(), "Pose goal receieved.");
	pose_goal_recv_ = true;
	linear_trajectory_recv_ = false;
	joint_space_goal_recv_ = false;

	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	//! STOMP planner accepts only joint-space goals!
	move_group.setPlanningPipelineId("stomp");

	move_group.startStateMonitor(2.0);
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	moveit::core::RobotState start_state(*move_group.getCurrentState());
	move_group.setStartState(start_state);

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// for (uint i = 0; i < NUM_JOINTS; i++)
	// 	RCLCPP_INFO(node_->get_logger(), "Current J%d angle: %f", i + 1, joint_group_positions[i]);

	moveit::core::RobotState goal_state(*move_group.getCurrentState());
	
	if (!goal_state.setFromIK(joint_model_group, goal_msg->pose))
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed to setfromik");
		return;
	} 

	goal_state.copyJointGroupPositions(joint_model_group, joint_group_positions);

	float vel_scaling_factor = (float) (goal_msg->speed / 100.0);
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);

	// move_group.setPoseTarget(goal_msg->pose);
	bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
	if (!within_bounds)
	{
		RCLCPP_WARN(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
	}

	bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
}


void ArmMoveGroup::execute_cb_(const std_msgs::msg::Bool::SharedPtr execute_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	move_group.startStateMonitor(2.0);
	move_group.setStartStateToCurrentState();

	if (execute_msg->data)
	{
		if (linear_trajectory_recv_)
		{
			if (move_group.asyncExecute(trajectory_) == moveit::core::MoveItErrorCode::SUCCESS)
				RCLCPP_INFO(node_->get_logger(), "Linear trajectory motion plan executed!\n");
			else
				RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
		}
		else
		{
			if (move_group.asyncExecute(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
				RCLCPP_INFO(node_->get_logger(), "Motion plan executed!\n");
			else
				RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
		}
	}
	else
	{
		move_group.stop();
		RCLCPP_INFO(node_->get_logger(), "Motion plan execution stopped!\n");
	}

	joint_space_goal_recv_ = false;
	pose_goal_recv_ = false;
	linear_trajectory_recv_ = false;
}


void ArmMoveGroup::stop_cb_(const std_msgs::msg::Bool::SharedPtr stop_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	if (stop_msg->data)
	{
		move_group.stop();
		RCLCPP_INFO(node_->get_logger(), "Motion plan execution stopped!\n");
	}

	joint_space_goal_recv_ = false;
	pose_goal_recv_ = false;
	linear_trajectory_recv_ = false;
}


void ArmMoveGroup::clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg)
{
	if (clear_msg->data)
	{
		auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

		if (joint_space_goal_recv_)
		{
			RCLCPP_INFO(node_->get_logger(), "Clearing joint space target");

			// move_group.getCurrentState(10.0);
			move_group.startStateMonitor(5.0);

			std::vector<double> joint_group_positions;
			joint_group_positions = move_group.getCurrentJointValues();

			RCLCPP_INFO(node_->get_logger(), "Current joint values:\n");
			for (uint i = 0; i < NUM_JOINTS; i++)
				RCLCPP_INFO(node_->get_logger(), "J%d %f ", i, joint_group_positions[i]);

			move_group.setJointValueTarget(joint_group_positions);

			bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

			if (success)
				RCLCPP_INFO(node_->get_logger(), "\nMotion plan reset!\n");
			else
				RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");

			joint_space_goal_recv_ = false;
		}

		if (pose_goal_recv_)
		{
			RCLCPP_INFO(node_->get_logger(), "Clearing pose target\n");
			move_group.clearPoseTarget(move_group.getEndEffectorLink());
			pose_goal_recv_ = false;
		}
	}
}


void ArmMoveGroup::timer_cb_()
{
	RCLCPP_INFO(mg_node_->get_logger(), "Spinning");
	std::cout << "spinning\n"; 
}


void ArmMoveGroup::save_cb_(
	const std::shared_ptr<Save::Request> request, 
	std::shared_ptr<Save::Response> response)
{
	const std::string type = request->type;
	const std::string label = request->label;

	RCLCPP_INFO(node_->get_logger(), "Received save %s request.", type.c_str());


	if (!strcmp(type.c_str(), "pose"))
	{
		// Start move group node state monitor
		auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
		move_group.startStateMonitor(2.0);

		// Get current state
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		
		std::vector<double> current_joint_positions;
		const moveit::core::JointModelGroup* joint_model_group =
			move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		current_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

		// for (uint i = 0; i < NUM_JOINTS; i++)
		// 	RCLCPP_INFO(node_->get_logger(), "J%d: %f", (i + 1), joint_group_positions[i]);


		SerializedPose sp;
		sp.joint_positions = current_joint_positions;
	

		//* Serialize current pose data and save into json
		std::string file_name = POSE_DIR;
		file_name.append(label);
		file_name.append(".pose.json");

		RCLCPP_INFO(node_->get_logger(), "Saving pose into: %s", file_name.c_str());

		{
			std::ofstream os(file_name.c_str());
			cereal::JSONOutputArchive oa(os);

			oa( cereal::make_nvp(label.c_str(), sp) );
		}


		// std::vector<double> joint_group_positions_temp;
		SerializedPose sp_temp;
		{
			std::ifstream is(file_name.c_str(), std::ios::in);
			cereal::JSONInputArchive ia(is);

			ia( sp_temp );
		}


		RCLCPP_INFO(node_->get_logger(), "Reading back saved pose: ");
		for (uint i = 0; i < NUM_JOINTS; i++)
			RCLCPP_INFO(node_->get_logger(), "J%d: %f", (i + 1), sp_temp.joint_positions[i]);
			

		if (!memcmp(&current_joint_positions.front(), &sp_temp.joint_positions.front(), (size_t) NUM_JOINTS))
		{
			RCLCPP_INFO(node_->get_logger(), "Pose successfully saved into %s", file_name.c_str());
			response->saved = true;
		}
		else
		{
			RCLCPP_INFO(node_->get_logger(), "Saving pose failed.");
			response->saved = false;
		}

	}
	else if (!strcmp(type.c_str(), "trajectory"))
	{
		size_t num_points = plan_.trajectory.joint_trajectory.points.size();
		
		// Create SerializedTrajectory obj and allocate appropriate size for vectors
		SerializedTrajectory st;
		st.points.resize(num_points);
		st.joint_names.resize(NUM_JOINTS);
		st.sec.resize(num_points);
		st.nanosec.resize(num_points);

		RCLCPP_INFO(node_->get_logger(), "Saving trajectory with %lu points.", num_points);

		// Copy over joint names
		std::vector< std::string > joint_names_ = plan_.trajectory.joint_trajectory.joint_names;
		for (uint i = 0; i < NUM_JOINTS; i++)
			st.joint_names[i] = joint_names_[i];

		// Copy over point joint positions and time stamps
		for (uint i = 0; i < num_points; i++)
		{
			st.points[i].reserve(NUM_JOINTS);
			st.points[i] = plan_.trajectory.joint_trajectory.points[i].positions;
			st.sec[i] = plan_.trajectory.joint_trajectory.points[i].time_from_start.sec;
			st.nanosec[i] = plan_.trajectory.joint_trajectory.points[i].time_from_start.nanosec;
		}

		// Serialize trajectory object and save into binary
		std::string file_name = TRAJ_DIR;
		file_name.append(label);
		file_name.append(".trajectory");
		{
			std::ofstream os(file_name.c_str(), std::ios::binary);
			cereal::BinaryOutputArchive oa(os);

			oa( st );
		}

		response->saved = true;
	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Save request unrecognized!");
		response->saved = false;
	}
}


void ArmMoveGroup::execute_saved_cb_(
	const std::shared_ptr<MoveToSaved::Request> request, 
	std::shared_ptr<MoveToSaved::Response> response)
{
	// Start move group interface
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);


	// Pose or trajectory
	const std::string type = request->type;

	// Get user-selected pose/trajectory
	const std::string label = request->label;


	if (!strcmp(type.c_str(), "pose"))
	{
		std::string file_path = POSE_DIR;
		file_path.append(label);
		file_path.append(".pose.json");


		// TODO: make sure file exists before loading archive
		// Create and load serialized pose obj
		SerializedPose sp;
		{
			std::ifstream is(file_path.c_str(), std::ios::in);
			cereal::JSONInputArchive ia(is);

			ia( sp );
		}

		bool within_bounds = move_group.setJointValueTarget(sp.joint_positions);
		if (!within_bounds)
		{
			RCLCPP_WARN(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
		}

		bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success)
		{
			RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
			response->executed = true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
			response->executed = false;
		}
	}
	else if (!strcmp(type.c_str(), "trajectory"))
	{
		std::string file_path = TRAJ_DIR;
		file_path.append(label);
		file_path.append(".trajectory");

		// TODO: make sure file exists before loading archive
		// Read saved trajectory
		SerializedTrajectory st;
		{
			std::ifstream is(file_path.c_str(), std::ios::binary);
			cereal::BinaryInputArchive ia(is);

			ia( st );
		}

		RCLCPP_INFO(node_->get_logger(), "Loaded trajectory with %lu points", st.points.size());
		

		moveit_msgs::msg::RobotTrajectory trajectory;
		trajectory.joint_trajectory.joint_names.resize(NUM_JOINTS);
		trajectory.joint_trajectory.points.resize(st.points.size());

		// RCLCPP_INFO(node_->get_logger(), "Setting frame_id: %s", move_group.getPlanningFrame().c_str());
		trajectory.joint_trajectory.header.frame_id = move_group.getPlanningFrame();

		// RCLCPP_INFO(node_->get_logger(), "Setting stamp: %f", node_->now().seconds());
		trajectory.joint_trajectory.header.stamp = node_->now();

		for (uint i = 0; i < NUM_JOINTS; i++)
			trajectory.joint_trajectory.joint_names[i] = st.joint_names[i];


		for (uint i = 0; i < st.points.size(); i++)
		{
			trajectory.joint_trajectory.points[i].positions = st.points[i];
			trajectory.joint_trajectory.points[i].time_from_start.sec = st.sec[i];
			trajectory.joint_trajectory.points[i].time_from_start.nanosec = st.nanosec[i];
		}

		RCLCPP_INFO(node_->get_logger(), "Attempting to execute saved trajectory...");
		
		// TODO: preview motion plan before moving

		if (move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS)
		{
			RCLCPP_INFO(node_->get_logger(), "Success!");
			response->executed = true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Execution failed");
			response->executed = false;
		}

	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Invalid type!");
		response->executed = false;
	}

}



int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto arm_move_group = ArmMoveGroup();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(arm_move_group.mg_node_);
	executor.add_node(arm_move_group.node_);
	// std::thread([&executor]() { executor.spin(); }).detach();

	try
	{
		// rclcpp::spin(arm_move_group.node_);
		executor.spin();
	}
	catch(const std::exception& e)
	{
		std::cerr << "Exception caught during spin: " << e.what() << '\n';
	}

	rclcpp::shutdown();
	return 0;
}
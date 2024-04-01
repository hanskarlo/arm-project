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

	collision_obj_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/SetupLabEnvironment",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::coll_obj_cb_, this, std::placeholders::_1)
	);

	joint_space_sub_ = node_->create_subscription<zeroerr_msgs::msg::JointSpaceTarget>(
		"arm/JointSpaceGoal",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::joint_space_cb_, this, std::placeholders::_1)
	);

	pose_array_sub_ = node_->create_subscription<zeroerr_msgs::msg::PoseTargetArray>(
		"arm/PoseGoalArray",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::pose_array_cb_, this, std::placeholders::_1)
	);

	pose_sub_ = node_->create_subscription<zeroerr_msgs::msg::PoseTarget>(
		"arm/PoseGoal",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::pose_cb_, this, std::placeholders::_1)
	);

	arm_execute_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Execute",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::execute_cb_, this, std::placeholders::_1)
	);

	arm_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Stop",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::stop_cb_, this, std::placeholders::_1)
	);

	arm_clear_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Clear",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::clear_cb_, this, std::placeholders::_1)
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


void ArmMoveGroup::joint_space_cb_(zeroerr_msgs::msg::JointSpaceTarget::SharedPtr goal_msg)
{
	RCLCPP_INFO(node_->get_logger(), "Joint space goal received.");
	joint_space_goal_recv_ = true;
	linear_trajectory_recv_ = false;
	pose_goal_recv_ = false;

	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	move_group.setPlanningPipelineId("stomp");

	move_group.startStateMonitor(2.0);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	move_group.setStartState(*current_state);

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (uint i = 0; i < NUM_JOINTS; i++)
		joint_group_positions[i] = ((goal_msg->joint_deg[i] * PI) / 180); // Deg -> Rad
	// RCLCPP_INFO(node_->get_logger(), "Converted deg to rad!\n");


	float vel_scaling_factor = (float) (goal_msg->speed / 100.0);
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
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
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
#include <arm_move_group/arm_move_group.h>

ArmMoveGroup::ArmMoveGroup()
{
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	mg_node_ = rclcpp::Node::make_shared("aro_movegroup_", node_options);
	node_ = rclcpp::Node::make_shared(NODE_NAME, node_options);


	using namespace std::placeholders;

	arm_clear_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Clear",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::clear_cb_, this, _1)
	);

	
	execute_srv_ = node_->create_service<Trigger>(
		"arm/Execute",
		std::bind(&ArmMoveGroup::execute_cb_, this, _1, _2)
	);

	stop_srv_ = node_->create_service<Trigger>(
		"arm/Stop",
		std::bind(&ArmMoveGroup::stop_cb_, this, _1, _2)
	);

	joint_space_goal_srv_ = node_->create_service<JointSpaceGoal>(
		"arm/JointSpaceGoal",
		std::bind(&ArmMoveGroup::joint_space_goal_cb_, this, _1, _2)
	);

	pose_goal_srv_ = node_->create_service<PoseGoal>(
		"arm/PoseGoal",
		std::bind(&ArmMoveGroup::pose_goal_cb_, this, _1, _2)
	);

	pose_goal_array_srv_ = node_->create_service<PoseGoalArray>(
		"arm/PoseGoalArray",
		std::bind(&ArmMoveGroup::pose_goal_array_cb_, this, _1, _2)
	);

	save_srv_ = node_->create_service<Save>(
		"arm/Save", 
		std::bind(&ArmMoveGroup::save_cb_, this, _1, _2)
	);

	move_to_saved_srv_ = node_->create_service<MoveToSaved>(
		"arm/ExecuteSaved",
		std::bind(&ArmMoveGroup::execute_saved_cb_, this, _1, _2)
	);

	get_state_srv_ = node_->create_service<GetState>(
		"arm/GetState",
		std::bind(&ArmMoveGroup::get_state_cb_, this, _1, _2)
	);


	// mg_node_->declare_parameter("visualize_trajectory", true);
	visualize_trajectories_ = node_->get_parameter("visualize_trajectory").as_bool();

	if (visualize_trajectories_)
		RCLCPP_INFO(node_->get_logger(), "Visualizing trajectories");
	else
		RCLCPP_INFO(node_->get_logger(), "Not visualizing trajectories");


	servoing_ = node_->get_parameter("servoing").as_bool();

	if (servoing_)
		RCLCPP_INFO(node_->get_logger(), "In servo mode");
	else
		RCLCPP_INFO(node_->get_logger(), "Not In servo mode");
	

	if (servoing_)
	{
		execution_feedback_sub_ = node_->create_subscription<ExecutionFeedback>(
			"/execute_trajectory/_action/feedback",
			rclcpp::QoS(1),
			std::bind(&ArmMoveGroup::exec_feedback_cb_, this, _1)
		);
	}


	RCLCPP_INFO(node_->get_logger(), "Initialized!");
}

ArmMoveGroup::~ArmMoveGroup()
{
	RCLCPP_INFO(node_->get_logger(), "Destruct sequence initiated.");
}


void ArmMoveGroup::exec_feedback_cb_(const ExecutionFeedback::SharedPtr feedback)
{
	std::string state = feedback->feedback.state;
	RCLCPP_INFO(node_->get_logger(), "Trajectory execution status: %s", feedback->feedback.state.c_str());

	if (strcmp(state.c_str(), "IDLE") == 0)
	{
		// Node for pausing servo_node if needed
		auto servo_pause_cli_node = rclcpp::Node::make_shared("pause_servo_cli_node_");
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_servo_cli_;
		pause_servo_cli_ = servo_pause_cli_node->create_client<std_srvs::srv::SetBool>("servo_node/pause_servo");
		
		auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
		req->data = false;

		auto future = pause_servo_cli_->async_send_request(req);

		if (rclcpp::spin_until_future_complete(servo_pause_cli_node, future) == rclcpp::FutureReturnCode::SUCCESS)
			RCLCPP_INFO(node_->get_logger(), "Called pause_servo service: %s", future.get()->message.c_str());
		else
			RCLCPP_ERROR(node_->get_logger(), "Failed to call pause_servo service");
	}
}



void ArmMoveGroup::joint_space_goal_cb_(
	const std::shared_ptr<JointSpaceGoal::Request> request, 
	std::shared_ptr<JointSpaceGoal::Response> response)
{
	RCLCPP_INFO(node_->get_logger(), "Joint space goal received.");
	

	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);


	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });


	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	
	// Start monitoring state of arm
	move_group.startStateMonitor(2.0);

	RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
	RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());


	// Use STOMP
	move_group.setPlanningPipelineId("stomp");

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
	move_group.setStartState(*current_state);

	const moveit::core::JointModelGroup* joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	
	const moveit::core::LinkModel* ee_link = 
		joint_model_group->getLinkModel(move_group.getEndEffectorLink());

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// for (uint i = 0; i < NUM_JOINTS; i++)
	// 	RCLCPP_INFO(node_->get_logger(), "J%d: %f", (i + 1), joint_group_positions[i]);


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
		if (visualize_trajectories_)
		{
			moveit_visual_tools::MoveItVisualTools visual_tools(
				move_group_node,
				move_group.getPlanningFrame(),
				"arm_marker_array",
				move_group.getRobotModel());

			visual_tools.deleteAllMarkers();

			bool visualized = visual_tools.publishTrajectoryLine(
				plan_.trajectory,
				ee_link,
				joint_model_group
			);

			visual_tools.trigger();
			
			if (visualized)
				RCLCPP_INFO(node_->get_logger(), "Motion plan visualized.");
			else
				RCLCPP_ERROR(node_->get_logger(), "Motion plan visualization failed\n");
		}

		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!");
		response->valid = true;
	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
		response->valid = false;
	}


	// Stop executor spin and join thread
	executor.cancel();
	t.join();
}


void ArmMoveGroup::pose_goal_cb_(
	const std::shared_ptr<PoseGoal::Request> request, 
	std::shared_ptr<PoseGoal::Response> response)
{
	RCLCPP_INFO(node_->get_logger(), "PoseGoal service called.");

	
	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);


	// Use STOMP
	move_group.setPlanningPipelineId("stomp");


	// Start monitoring state of arm
	move_group.startStateMonitor(2.0);
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	
	// Set start state to current state
	moveit::core::RobotState start_state(*move_group.getCurrentState());
	move_group.setStartState(start_state);

	
	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	const moveit::core::LinkModel* ee_link = 
		joint_model_group->getLinkModel(move_group.getEndEffectorLink());


	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// for (uint i = 0; i < NUM_JOINTS; i++)
	// 	RCLCPP_INFO(node_->get_logger(), "Current J%d angle: %f", i + 1, joint_group_positions[i]);


	moveit::core::RobotState goal_state(*move_group.getCurrentState());
	
	//* STOMP accepts only joint-space goals:
	// Get joint angles of request->pose using IK
	if (!goal_state.setFromIK(joint_model_group, request->pose))
	{
		RCLCPP_ERROR(node_->get_logger(), "Failed IK");
		response->valid = false;
		return;
	} 

	// Fill goal_state joint positions with joint positions calculated from setFromIk()
	goal_state.copyJointGroupPositions(joint_model_group, joint_group_positions);


	// Set speed/accel scaling factors
	float vel_scaling_factor = (float) (request->speed / 100.0);
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);


	bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
	if (!within_bounds)
		RCLCPP_WARN(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");


	// Generate motion plan from joint value targets
	bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);


	if (success)
	{
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
		response->valid = true;

		if (visualize_trajectories_)
		{
			moveit_visual_tools::MoveItVisualTools visual_tools(
				move_group_node,
				move_group.getPlanningFrame(),
				"arm_marker_array",
				move_group.getRobotModel());

			visual_tools.deleteAllMarkers();

			bool visualized = visual_tools.publishTrajectoryLine(
				plan_.trajectory,
				ee_link,
				joint_model_group
			);

			visual_tools.trigger();

			if (visualized)
				RCLCPP_INFO(node_->get_logger(), "Motion plan visualized.");
			else
				RCLCPP_ERROR(node_->get_logger(), "Motion plan visualization failed\n");
		}
	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
		response->valid = false;
	}


	// Stop executor spin and join thread
	executor.cancel();
	t.join();
}


void ArmMoveGroup::pose_goal_array_cb_(
	const std::shared_ptr<PoseGoalArray::Request> request, 
	std::shared_ptr<PoseGoalArray::Response> response)
{
	RCLCPP_INFO(node_->get_logger(), "PoseGoalArray service call received.");


	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	
	// Start monitoring state of arm
	move_group.startStateMonitor(2.0);
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	moveit::core::RobotState start_state(*move_group.getCurrentState());
	move_group.setStartState(start_state);

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	
	const moveit::core::LinkModel* ee_link = 
		joint_model_group->getLinkModel(move_group.getEndEffectorLink());

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


	// Get type of motion
	const std::string type = request->type;

	if (!strcmp(type.c_str(), "linear"))
	{
		linear_trajectory_recv_ = true;

		std::vector<geometry_msgs::msg::Pose> waypoints;
		// waypoints.push_back(move_group.getCurrentPose().pose);

		RCLCPP_INFO(node_->get_logger(), "Pose array receieved with %lu waypoints.", request->waypoints.size());
		// move_group.setMaxVelocityScalingFactor(0.1);

		for (size_t i = 0; i < request->waypoints.size(); i++)
			waypoints.push_back(request->waypoints[i]);
		
		// moveit_msgs::msg::RobotTrajectory trajectory;
		// const double jump_threshold = 0.0;	// Disable jump threshold
		// const double eef_step = 0.01; 		// 1cm interpolation resolution
		const double jump_threshold = request->jump_threshold;
		const double eef_step = request->step_size;
		double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_);

		// If percent of path achieved >= 95%
		if ((fraction * 100.0) >= 95.0 )
		{
			RCLCPP_INFO(node_->get_logger(), "Planning cartesian path (%.2f%% achieved)", fraction * 100.0);
			response->success = true;

			if (visualize_trajectories_)
			{
				moveit_visual_tools::MoveItVisualTools visual_tools(
					mg_node_,
					move_group.getPlanningFrame(),
					"arm_marker_array",
					move_group.getRobotModel());

				visual_tools.deleteAllMarkers();

				bool visualized = visual_tools.publishTrajectoryLine(
					trajectory_,
					ee_link,
					joint_model_group
				);

				visual_tools.trigger();
			
				if (visualized)
					RCLCPP_INFO(node_->get_logger(), "Motion plan visualized.");
				else
					RCLCPP_ERROR(node_->get_logger(), "Motion plan visualization failed\n");
			}
		}
		else
		{
			RCLCPP_WARN(node_->get_logger(), "Planning cartesian path (%.2f%% achieved)", fraction * 100.0);
			response->success = false;
		}
	}
	else if(!strcmp(type.c_str(), "arc"))
	{
		RCLCPP_INFO(node_->get_logger(), "Arc motion request received.");

		move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
		move_group.setPlannerId("CIRC");


		geometry_msgs::msg::PoseStamped center;
		geometry_msgs::msg::PoseStamped endpoint;

		// First pose in waypoint array used as arc center
		center.pose = request->waypoints[0];
		center.header.frame_id = "arm_Link";

		// Second pose used as arc endpoint
		endpoint.pose = request->waypoints[1];
		endpoint.header.frame_id = "arm_Link";

		// Set endpoint as pose target
		move_group.setPoseTarget(endpoint);
		
		// Add center of circular motion as path constraint
		moveit_msgs::msg::Constraints constraints;
		moveit_msgs::msg::PositionConstraint pos_constraint;
		constraints.name = "center";
		pos_constraint.header.frame_id = center.header.frame_id;
		pos_constraint.link_name = "j6_Link";
		pos_constraint.constraint_region.primitive_poses.push_back(center.pose);
		pos_constraint.weight = 1.0;
		constraints.position_constraints.push_back(pos_constraint);
		move_group.setPathConstraints(constraints);

		// Generate motion plan
		bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success)
		{
			RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
			response->success = true;

			if (visualize_trajectories_)
			{
				moveit_visual_tools::MoveItVisualTools visual_tools(
					mg_node_,
					move_group.getPlanningFrame(),
					"arm_marker_array",
					move_group.getRobotModel());

				visual_tools.deleteAllMarkers();

				bool visualized = visual_tools.publishTrajectoryLine(
					plan_.trajectory,
					ee_link,
					joint_model_group
				);

				visual_tools.trigger();
			
				if (visualized)
					RCLCPP_INFO(node_->get_logger(), "Motion plan visualized.");
				else
					RCLCPP_ERROR(node_->get_logger(), "Motion plan visualization failed\n");
			}
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
			response->success = false;
		}

		move_group.clearPathConstraints();
	}
	else
	{
		RCLCPP_ERROR(node_->get_logger(), "Unrecognized pose goal array type (%s), see PoseGoalArray.srv", type.c_str());
		response->success = false;
	}


	// Stop executor spin and join thread
	executor.cancel();
	t.join();
}


void ArmMoveGroup::stop_cb_(
	const std::shared_ptr<Trigger::Request> request, 
	std::shared_ptr<Trigger::Response> response)
{
	// Supress compiler warning
	const auto rq = request;

	RCLCPP_INFO(node_->get_logger(), "/arm/Stop service call received\n");

	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mg_stop_node", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	try
	{
		move_group.stop();
		RCLCPP_INFO(node_->get_logger(), "Motion plan execution stopped!\n");
		response->message = "Motion plan execution stopped!\n";
		response->success = true;
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(node_->get_logger(), "Exception caught attempting stop: %s", e.what());
		response->message = e.what();
		response->success = false;
	}

	// Stop executor spin and join thread
	executor.cancel();
	t.join();
}


void ArmMoveGroup::clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg)
{
	if (clear_msg->data)
	{
		auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
		move_group.startStateMonitor(5.0);

		if (joint_space_goal_recv_)
		{
			RCLCPP_INFO(node_->get_logger(), "Clearing joint space target");


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


void ArmMoveGroup::save_cb_(
	const std::shared_ptr<Save::Request> request, 
	std::shared_ptr<Save::Response> response)
{
	const std::string type = request->type;
	const std::string label = request->label;

	RCLCPP_INFO(node_->get_logger(), "Received save %s request.", type.c_str());


	if (!strcmp(type.c_str(), "pose"))
	{
		// Node for move group interface
		rclcpp::NodeOptions node_options;
		node_options.automatically_declare_parameters_from_overrides(true);
		auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

		// Add node to executor and spin in another thread
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(move_group_node);
		std::thread t([&executor]() { executor.spin(); });

		// Create move group interface using move_group_node
		auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
		// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
		
		
		// Start monitoring state of arm
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


		// Stop executor spin and join thread
		executor.cancel();
		t.join();

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


void ArmMoveGroup::execute_cb_(
	const std::shared_ptr<Trigger::Request> request, 
	std::shared_ptr<Trigger::Response> response)
{
	// Supress compiler warning
	(void) request;

	// Pause servo_node if running
	if (servoing_)
	{
		// Node for pausing servo_node if needed
		auto servo_pause_cli_node = rclcpp::Node::make_shared("pause_servo_cli_node_");
		
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_servo_cli_;
		pause_servo_cli_ = servo_pause_cli_node->create_client<std_srvs::srv::SetBool>("servo_node/pause_servo");
		
		auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
		req->data = true;

		auto future = pause_servo_cli_->async_send_request(req);

		if (rclcpp::spin_until_future_complete(servo_pause_cli_node, future) == rclcpp::FutureReturnCode::SUCCESS)
			RCLCPP_INFO(node_->get_logger(), "Called pause_servo service: %s", future.get()->message.c_str());
		else
			RCLCPP_ERROR(node_->get_logger(), "Failed to call pause_servo service");
	}


	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	
	// Start monitoring state of arm
	move_group.startStateMonitor(2.0);
	move_group.setStartStateToCurrentState();


	if (linear_trajectory_recv_)
	{
		if (move_group.asyncExecute(trajectory_) == moveit::core::MoveItErrorCode::SUCCESS)
		{
			RCLCPP_INFO(node_->get_logger(), "Linear trajectory motion plan executed!\n");
			response->message = "Linear trajectory motion plan executed!\n";
			response->success = true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
			response->message = "Motion execution failed";
			response->success = false;
		}
		
		linear_trajectory_recv_ = false;
	}
	else
	{
		if (move_group.asyncExecute(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
		{
			RCLCPP_INFO(node_->get_logger(), "Motion plan executed!\n");
			response->message = "Motion plan executed!\n";
			response->success = true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
			response->message = "Motion execution failed";
			response->success = false;
		}
	}

	// Stop executor spin and join thread
	executor.cancel();
	t.join();

}


void ArmMoveGroup::execute_saved_cb_(
	const std::shared_ptr<MoveToSaved::Request> request, 
	std::shared_ptr<MoveToSaved::Response> response)
{
	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);


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


	// Stop executor spin and join thread
	executor.cancel();
	t.join();

}


void ArmMoveGroup::get_state_cb_(
	const std::shared_ptr<GetState::Request> request, 
	std::shared_ptr<GetState::Response> response)
{
	// Suppress compiler warning
	(void) request;

	RCLCPP_INFO(node_->get_logger(), "Received GetState service call.");


	// Node for move group interface
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("mgn", node_options);

	// Add node to executor and spin in another thread
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread t([&executor]() { executor.spin(); });

	// Create move group interface using move_group_node
	auto move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);
	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	
	// Start monitoring state of arm
	move_group.startStateMonitor(2.0);


	// Get end effector coordinates in arm_link frame
	response->coordinates.x = move_group.getCurrentPose().pose.position.x;
	response->coordinates.y = move_group.getCurrentPose().pose.position.y;
	response->coordinates.z = move_group.getCurrentPose().pose.position.z;

	// RCLCPP_INFO(node_->get_logger(), "x: %f", response->coordinates.x);
	// RCLCPP_INFO(node_->get_logger(), "y: %f", response->coordinates.y);
	// RCLCPP_INFO(node_->get_logger(), "z: %f", response->coordinates.z);

	
	// Get joint positions, converted to degrees
	for (uint i = 0; i < NUM_JOINTS; i++)
	{
		int16_t deg = (int16_t) (move_group.getCurrentJointValues()[i] * 180.0) / PI;
		response->joint_pos_deg[i] = deg;

		// RCLCPP_INFO(node_->get_logger(), "J%d: %d degrees", i, deg);
	}

	// Stop executor spin and join thread
	executor.cancel();
	t.join();
}




int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto arm_move_group = ArmMoveGroup();

	rclcpp::executors::MultiThreadedExecutor executor;
	// executor.add_node(arm_move_group.mg_node_);
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
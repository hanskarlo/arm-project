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

	// auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	
	// RCLCPP_INFO(node_->get_logger(), "Initializing ArmMoveGroup");
	// RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
	// RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

	// // std::vector<double> current_joint_pos = move_group.getCurrentJointValues();

	// // RCLCPP_INFO(node_->get_logger(), "Current joint positions:");
	// // for (uint i = 0; i < NUM_JOINTS; i++)
	// // 	RCLCPP_INFO(node_->get_logger(), "J%d: %f", i, current_joint_pos[i]);

	// //* Place table underneath arm
	// table_.header.frame_id = move_group.getPlanningFrame();

	// table_.id = "table1";

	// shape_msgs::msg::SolidPrimitive primitive;
	// primitive.type = primitive.BOX;
	// primitive.dimensions.resize(3);
	// primitive.dimensions[primitive.BOX_X] = 0.91;
	// primitive.dimensions[primitive.BOX_Y] = 1.54;
	// primitive.dimensions[primitive.BOX_Z] = 0.08;

	// geometry_msgs::msg::Pose table_pose;
	// table_pose.orientation.w = 1.0;
	// table_pose.position.x = 0.405;
	// table_pose.position.y = -0.67;
	// table_pose.position.z = -0.07;

	// table_.primitives.push_back(primitive);
	// table_.primitive_poses.push_back(table_pose);
	// table_.operation = table_.ADD;

	// planning_scene_interface_.applyCollisionObject(table_);


	arm_joint_space_sub_ = node_->create_subscription<zeroerr_msgs::msg::JointSpaceTarget>(
		"arm/JointSpaceGoal",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::arm_joint_space_cb_, this, std::placeholders::_1)
	);

	arm_point_sub_ = node_->create_subscription<zeroerr_msgs::msg::PoseTarget>(
		"arm/PoseGoal",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::arm_pose_cb_, this, std::placeholders::_1)
	);

	arm_execute_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Execute",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::arm_execute_cb_, this, std::placeholders::_1)
	);

	arm_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Stop",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::arm_stop_cb_, this, std::placeholders::_1)
	);

	arm_clear_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
		"arm/Clear",
		rclcpp::QoS(1),
		std::bind(&ArmMoveGroup::arm_clear_cb_, this, std::placeholders::_1)
	);

	RCLCPP_INFO(node_->get_logger(), "Initialized!");
}

ArmMoveGroup::~ArmMoveGroup()
{
	RCLCPP_INFO(node_->get_logger(), "Destruct sequence initiated.");

	table_.operation = table_.REMOVE;
	planning_scene_interface_.applyCollisionObject(table_);
}


void ArmMoveGroup::arm_joint_space_cb_(zeroerr_msgs::msg::JointSpaceTarget::SharedPtr goal_msg)
{
	RCLCPP_INFO(node_->get_logger(), "Joint space goal received.");
	joint_space_goal_recv_ = true;

	float vel_scaling_factor = (float) (goal_msg->speed / 100.0);
	// RCLCPP_INFO(node_->get_logger(), "Velocity scaling factor: %f(%u/100)", vel_scaling_factor, goal_msg->speed);
	
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (uint i = 0; i < NUM_JOINTS; i++)
		joint_group_positions[i] = ((goal_msg->joint_deg[i] * PI) / 180); // Deg -> Rad
	// RCLCPP_INFO(node_->get_logger(), "Converted deg to rad!\n");

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



void ArmMoveGroup::arm_pose_cb_(zeroerr_msgs::msg::PoseTarget::SharedPtr goal_msg)
{
	RCLCPP_INFO(node_->get_logger(), "Pose goal receieved.");
	pose_goal_recv_ = true;

	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	float vel_scaling_factor = goal_msg->speed / 100;
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);

	move_group.setPoseTarget(goal_msg->pose);

	bool success = (move_group.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
}


void ArmMoveGroup::arm_execute_cb_(const std_msgs::msg::Bool::SharedPtr execute_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	if (execute_msg->data)
	{
		if (move_group.asyncExecute(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
			RCLCPP_INFO(node_->get_logger(), "Motion plan executed!\n");
		else
			RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
	}
	else
	{
		move_group.stop();
		RCLCPP_INFO(node_->get_logger(), "Motion plan execution stopped!\n");
	}

	joint_space_goal_recv_ = false;
	pose_goal_recv_ = false;
}


void ArmMoveGroup::arm_stop_cb_(const std_msgs::msg::Bool::SharedPtr stop_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(mg_node_, PLANNING_GROUP);

	if (stop_msg->data)
	{
		move_group.stop();
		RCLCPP_INFO(node_->get_logger(), "Motion plan execution stopped!\n");
	}

	joint_space_goal_recv_ = false;
	pose_goal_recv_ = false;
}

void ArmMoveGroup::arm_clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg)
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
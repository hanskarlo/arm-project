#include <zeroerr_move_group/zeroerr_move_group.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)

ArmMoveGroup::ArmMoveGroup()
{
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);
	node_ = rclcpp::Node::make_shared("arm_move_group", options);


	auto move_group = moveit::planning_interface::MoveGroupInterface(node_, PLANNING_GROUP);
	// move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group);
	
	RCLCPP_INFO(node_->get_logger(), "Initializing ArmMoveGroup");
	RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
	RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
	


	arm_joint_space_sub_ = node_->create_subscription<zeroerr_msgs::msg::ArmJointSpace>(
		"arm/jointspace",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::arm_joint_space_cb_, this, std::placeholders::_1)
	);

	arm_point_sub_ = node_->create_subscription<zeroerr_msgs::msg::ArmPoint>(
		"arm/point",
		rclcpp::QoS(10),
		std::bind(&ArmMoveGroup::arm_point_cb_, this, std::placeholders::_1)
	);
}

ArmMoveGroup::~ArmMoveGroup()
{
	RCLCPP_INFO(node_->get_logger(), "Destruct sequence initiated.");
}


void ArmMoveGroup::arm_joint_space_cb_(zeroerr_msgs::msg::ArmJointSpace::SharedPtr goal_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(node_, PLANNING_GROUP);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (uint i = 0; i < NUM_JOINTS; i++)
		joint_group_positions[i] = ((goal_msg->joint_pos_deg[i] * PI) / 180); // Deg -> Rad
	RCLCPP_INFO(node_->get_logger(), "Converted deg to rad!\n");

	bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
	if (!within_bounds)
	{
		RCLCPP_WARN(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
	}
	RCLCPP_INFO(node_->get_logger(), "Motion plan within bounds!\n");

	float vel_scaling_factor = goal_msg->speed / 100;
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);
	RCLCPP_INFO(node_->get_logger(), "Set scaling factors!\n");


	bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
	
	success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
		RCLCPP_INFO(node_->get_logger(), "Motion plan executed!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion execution failed\n");
}



void ArmMoveGroup::arm_point_cb_(zeroerr_msgs::msg::ArmPoint::SharedPtr goal_msg)
{
	auto move_group = moveit::planning_interface::MoveGroupInterface(node_, PLANNING_GROUP);

	// const moveit::core::JointModelGroup* joint_model_group =
    //   move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	float vel_scaling_factor = goal_msg->speed / 100;
	move_group.setMaxVelocityScalingFactor(vel_scaling_factor);
	move_group.setMaxAccelerationScalingFactor(0.5);

	move_group.setPoseTarget(goal_msg->pose);

	bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
		RCLCPP_INFO(node_->get_logger(), "Motion plan successful!\n");
	else
		RCLCPP_ERROR(node_->get_logger(), "Motion plan failed\n");
}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	auto arm_move_group = ArmMoveGroup();

	// arm_move_group.node_ = rclcpp::Node::make_shared("arm_move_group", node_options);

	// rclcpp::executors::SingleThreadedExecutor executor;
	// executor.add_node(move_group_node);
	// std::thread([&executor]() { executor.spin(); }).detach();

	try
	{
		rclcpp::spin(arm_move_group.node_);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	

	rclcpp::shutdown();
	return 0;
}
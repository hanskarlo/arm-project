#include <chrono>
#include <thread>

#include <fstream>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/access.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arm_msgs/srv/joint_space_goal.hpp"
#include "arm_msgs/srv/pose_goal.hpp"
#include "arm_msgs/srv/pose_goal_array.hpp"
#include "arm_msgs/srv/save.hpp"
#include "arm_msgs/srv/move_to_saved.hpp"
#include "arm_msgs/srv/get_state.hpp"

#include <moveit_msgs/action/execute_trajectory.hpp>


using namespace std::chrono_literals;

class ArmMoveGroup
{
    public:
        ArmMoveGroup();
        ~ArmMoveGroup();

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr mg_node_;



    private:
        const uint NUM_JOINTS = 6;
        const float PI = 3.141592654;
        const std::string PLANNING_GROUP = "arm_group";
        const std::string NODE_NAME = "arm_move_group";

        const std::string PKG_DIR = "/home/arodev0/arm_ws/src/zeroerr_arm/zeroerr_move_group/";
        const std::string POSE_DIR = PKG_DIR + "poses/";
        const std::string TRAJ_DIR = PKG_DIR + "trajectories/";

        //* ROS2 Parameters
        bool visualize_trajectories_ = true;
        bool servoing_ = false;

        bool in_execution_ = false;
        bool joint_space_goal_recv_ = false;
        bool pose_goal_recv_ = false;
        bool linear_trajectory_recv_ = false;



        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_obj_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_clear_sub_; 

        using ExecutionFeedback = moveit_msgs::action::ExecuteTrajectory_FeedbackMessage;
        rclcpp::Subscription<ExecutionFeedback>::SharedPtr execution_feedback_sub_;


        moveit_msgs::msg::RobotTrajectory trajectory_;

        void clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg);
        void exec_feedback_cb_(const ExecutionFeedback::SharedPtr feedback);

        // Feature-set services
        using JointSpaceGoal = arm_msgs::srv::JointSpaceGoal;
        using PoseGoal = arm_msgs::srv::PoseGoal;
        using PoseGoalArray = arm_msgs::srv::PoseGoalArray;
        using Trigger = std_srvs::srv::Trigger;
        using Save = arm_msgs::srv::Save;
        using MoveToSaved = arm_msgs::srv::MoveToSaved;
        using GetState = arm_msgs::srv::GetState;

        rclcpp::Service<Trigger>::SharedPtr execute_srv_;
        rclcpp::Service<Trigger>::SharedPtr stop_srv_;
        rclcpp::Service<JointSpaceGoal>::SharedPtr joint_space_goal_srv_;
        rclcpp::Service<PoseGoal>::SharedPtr pose_goal_srv_;
        rclcpp::Service<PoseGoalArray>::SharedPtr pose_goal_array_srv_;
        rclcpp::Service<Save>::SharedPtr save_srv_;
        rclcpp::Service<MoveToSaved>::SharedPtr move_to_saved_srv_;  
        rclcpp::Service<GetState>::SharedPtr get_state_srv_;

        rclcpp::Client<Trigger>::SharedPtr pause_servo_input_cli_;


        // Service callbacks
        void save_cb_(const std::shared_ptr<Save::Request> request, std::shared_ptr<Save::Response> response);
        void execute_cb_(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
        void execute_saved_cb_(const std::shared_ptr<MoveToSaved::Request> request, std::shared_ptr<MoveToSaved::Response> response);
        void stop_cb_(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
        void joint_space_goal_cb_(const std::shared_ptr<JointSpaceGoal::Request> request, std::shared_ptr<JointSpaceGoal::Response> response);
        void pose_goal_cb_(const std::shared_ptr<PoseGoal::Request> request, std::shared_ptr<PoseGoal::Response> response);
        void pose_goal_array_cb_(const std::shared_ptr<PoseGoalArray::Request> request, std::shared_ptr<PoseGoalArray::Response> response);
        void get_state_cb_(const std::shared_ptr<GetState::Request> request, std::shared_ptr<GetState::Response> response);


        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;




        struct SerializedPose
        {
            std::vector<double> joint_positions;

            template <class Archive>
            void serialize(Archive & archive)
            {
                archive( CEREAL_NVP(joint_positions) );
            }
        
        };

        struct SerializedTrajectory
        {   
            // Joint trajectory positions
            std::vector< std::vector<double> > points;

            // Joint names
            std::vector< std::string > joint_names;

            // Time from start
            std::vector<int32_t> sec;
            std::vector<uint32_t> nanosec;

            template<class Archive>
            void serialize(Archive & archive)
            {
                archive( points, joint_names, sec, nanosec );
            }
        };

};
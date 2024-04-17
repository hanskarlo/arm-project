#include <chrono>
#include <thread>

#include <fstream>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/access.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <zeroerr_msgs/action/move_to_saved.hpp>

#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "zeroerr_msgs/msg/collision_object.hpp"
#include "zeroerr_msgs/msg/joint_space_target.hpp"
#include "zeroerr_msgs/msg/pose_target_array.hpp"
#include "zeroerr_msgs/msg/pose_target.hpp"

#include "zeroerr_msgs/srv/save_motion.hpp"
#include "zeroerr_msgs/srv/move_to_saved.hpp"

using namespace std::chrono_literals;

class ArmMoveGroup
{
    public:
        ArmMoveGroup();
        ~ArmMoveGroup();

        void begin_();

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr mg_node_;



    private:
        const uint NUM_JOINTS = 6;
        const float PI = 3.141592654;
        const std::string PLANNING_GROUP = "arm_group";
        const std::string NODE_NAME = "arm_move_group";

        const std::string PKG_DIR = "/src/zeroerr_arm/zeroerr_move_group";
        const std::string POSE_DIR = PKG_DIR + "/poses";
        const std::string TRAJ_DIR = PKG_DIR + "/trajectories";

        bool joint_space_goal_recv_ = false;
        bool pose_goal_recv_ = false;
        bool linear_trajectory_recv_ = false;

        // rclcpp::Subscription<zeroerr_msgs::msg::CollisionObject>::SharedPtr collision_obj_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_obj_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::JointSpaceTarget>::SharedPtr joint_space_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::PoseTargetArray>::SharedPtr pose_array_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::PoseTarget>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_execute_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_stop_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_clear_sub_;      

        moveit_msgs::msg::RobotTrajectory trajectory_;

        // void coll_obj_cb_(const zeroerr_msgs::msg::CollisionObject::SharedPtr coll_obj_msg);
        void coll_obj_cb_(const std_msgs::msg::Bool::SharedPtr coll_obj_msg);
        void joint_space_cb_(zeroerr_msgs::msg::JointSpaceTarget::SharedPtr goal_msg);
        void pose_array_cb_(zeroerr_msgs::msg::PoseTargetArray::SharedPtr pose_array_msg);
        void pose_cb_(zeroerr_msgs::msg::PoseTarget::SharedPtr goal_msg);
        void execute_cb_(const std_msgs::msg::Bool::SharedPtr execute_msg);
        void stop_cb_(const std_msgs::msg::Bool::SharedPtr stop_msg);
        void clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg);
        void timer_cb_();


        //* Feature-set services
        using SaveMotion = zeroerr_msgs::srv::SaveMotion;
        using MoveToSaved = zeroerr_msgs::srv::MoveToSaved;
        rclcpp::Service<SaveMotion>::SharedPtr save_pose_srv_;
        rclcpp::Service<SaveMotion>::SharedPtr save_traj_srv_;
        rclcpp::Service<MoveToSaved>::SharedPtr move_to_saved_srv_;  


        // Service callbacks
        void save_pose_(const std::shared_ptr<SaveMotion::Request> request, std::shared_ptr<SaveMotion::Response> response);
        void save_traj_(const std::shared_ptr<SaveMotion::Request> request, std::shared_ptr<SaveMotion::Response> response);
        void execute_saved_(const std::shared_ptr<MoveToSaved::Request> request, std::shared_ptr<MoveToSaved::Response> response);

        zeroerr_msgs::msg::JointSpaceTarget arm_joint_space_cmd_;
        zeroerr_msgs::msg::PoseTarget arm_point_cmd_;
        moveit_msgs::msg::CollisionObject table_;


        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;


        struct SerializedTrajectory
        {   
            // Joint trajectory positions
            std::vector< std::vector<double> > points;

            // Time from start
            std::vector<int32_t> sec;
            std::vector<uint32_t> nanosec;

            template<class Archive>
            void serialize(Archive & archive)
            {
                archive( points, sec, nanosec );
            }
        };

};
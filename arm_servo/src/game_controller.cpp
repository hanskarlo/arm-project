#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string JOY_TOPIC   = "/joy";
const std::string JOY_FB_TOPIC  = "/joy/set_feedback";
const std::string PLANNING_FRAME_ID = "arm_Link";
const std::string EE_FRAME_ID = "j6_Link";
const size_t ROS_QUEUE_SIZE = 10;
const int NUM_JOINTS = 6;

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X  = 0,
  LEFT_STICK_Y  = 1,
  RIGHT_STICK_X = 2,
  RIGHT_STICK_Y = 3,
  LEFT_TRIGGER  = 4,
  RIGHT_TRIGGER = 5,
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  SELECT = 4,
  GUIDE = 5,
  MENU = 6,
  LEFT_STICK_CLICK = 7,
  RIGHT_STICK_CLICK = 8,
  LEFT_BUMPER = 9,
  RIGHT_BUMPER = 10,
  DPAD_UP = 11,
  DPAD_DOWN = 12,
  DPAD_LEFT = 13,
  DPAD_RIGHT = 14,
};

#define PRESSED 1

class GameController
{
    public:
        GameController();
        rclcpp::Node::SharedPtr nh_;

    private:
        rclcpp::Node::SharedPtr service_node_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
        
        rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_fb_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_cmd_type_cli_;
        std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> servo_command_type_;
        double joint_vel_cmd_;       // rad/s
        double cartesian_step_size_; // meters
        std::string command_frame_id_;

        // Flag to enable rising-edge triggering of buttons
        bool servo_cmd_toggle_;
        bool right_bumper_toggle_;
        bool left_bumper_toggle_;
        bool dpad_toggle_;

        enum Plane{
            XY,
            XZ,
            YZ
        };

        // Track joint being controlled during JointJog
        int joint_num_;

        void joy_cb_(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
};

GameController::GameController() : 
    joint_vel_cmd_(0.1), 
    cartesian_step_size_(0.1), 
    command_frame_id_{"arm_Link"},
    servo_cmd_toggle_(true),
    right_bumper_toggle_(true),
    left_bumper_toggle_(true),
    cartesian_plane_toggle_(true),
    joint_num_(0)
{
    // Node bridging Joy with MoveIt2 Servo
    nh_ = rclcpp::Node::make_shared("servo_game_controller");
    service_node_ = rclcpp::Node::make_shared("servo_game_controller_sn_");

    RCLCPP_INFO(nh_->get_logger(), "MoveIt2 Servo via Game Controller");

    // JointJog and CartesianJog topic publishers
    joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);

    // Joy feedback publisher
    joy_fb_pub_ = nh_->create_publisher<sensor_msgs::msg::JoyFeedback>(JOY_FB_TOPIC, ROS_QUEUE_SIZE);
    
    // Joy topic subscriber
    joy_sub_ = nh_->create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, ROS_QUEUE_SIZE, std::bind(&GameController::joy_cb_, this, std::placeholders::_1));

    // Client for switching input types, start in JointJog mode by default
    servo_command_type_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    servo_command_type_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    servo_cmd_type_cli_ = service_node_->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/switch_command_type");
    servo_cmd_type_cli_->wait_for_service(std::chrono::seconds(2));
    servo_cmd_type_cli_->async_send_request(servo_command_type_);

    RCLCPP_INFO(service_node_->get_logger(), "Servo mode starting in JointJog");
}


void GameController::joy_cb_(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{

    // MENU button used to toggle b/e Joint and Cartesian jogging modes
    if ((joy_msg->buttons[MENU] == PRESSED) && servo_cmd_toggle_)
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(service_node_);
        std::thread t([&executor]() { executor.spin(); });

        if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
        {
            servo_command_type_->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
            auto future = servo_cmd_type_cli_->async_send_request(servo_command_type_);

            // rclcpp::spin_until_future_complete(service_node_, future);

            RCLCPP_INFO(service_node_->get_logger(), "Servo command type switch to CartesianJog");
        }
        else if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::TWIST)
        {
            servo_command_type_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
            auto future = servo_cmd_type_cli_->async_send_request(servo_command_type_);

            // rclcpp::spin_until_future_complete(service_node_, future);

            RCLCPP_INFO(service_node_->get_logger(), "Servo command type switch to JointJog");
        }

        executor.cancel();
        t.join();
        // RCLCPP_INFO(service_node_->get_logger(), "Joined service node");

        servo_cmd_toggle_ = false;
        return;
    }
    else if ((joy_msg->buttons[MENU] == !PRESSED) && !servo_cmd_toggle_)
    {
        servo_cmd_toggle_ = true;
        return;
    }



    // DPAD UP/DOWN used to increase/decrease speed
    if (joy_msg->buttons[DPAD_UP] == PRESSED && dpad_toggle_)
    {
        if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
        {
            joint_vel_cmd_ += 0.1;
            RCLCPP_INFO(nh_->get_logger(), "JointJog speed increased: %frad/s", joint_vel_cmd_);
        }
        else
        {
            cartesian_step_size_ += 0.01;
            RCLCPP_INFO(nh_->get_logger(), "CartesianJog step increased: %fcm", (cartesian_step_size_ * 10));
        }

        dpad_toggle_ = false;
        return;
    }
    else if (joy_msg->buttons[DPAD_DOWN] == PRESSED && dpad_toggle_)
    {   
        if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
        {
            joint_vel_cmd_ -= 0.1;
            
            if (joint_vel_cmd_ < 0)
            {
                joint_vel_cmd_ = 0.1;
                RCLCPP_WARN(nh_->get_logger(), "JointJog speed minimum reached: %frad/s", joint_vel_cmd_);
            }
            else
            {
                RCLCPP_INFO(nh_->get_logger(), "JointJog speed decreased: %frad/s", joint_vel_cmd_);
            }
        }
        else
        {
            cartesian_step_size_ -= 0.01;
            if (cartesian_step_size_ < 0)
            {
                cartesian_step_size_ = 0.01;
                RCLCPP_INFO(nh_->get_logger(), "CartesianJog step size minimum reached: %fcm", (cartesian_step_size_ * 10));
            }
            else
            {
                RCLCPP_INFO(nh_->get_logger(), "CartesianJog step decreased: %fcm", (cartesian_step_size_ * 10));
            }
        }
        
        dpad_toggle_ = false;
        return;
    }

    // Reset DPAD rising edge trigger flag
    if ((  joy_msg->buttons[DPAD_UP]
        || joy_msg->buttons[DPAD_DOWN]
        || joy_msg->buttons[DPAD_LEFT]
        || joy_msg->buttons[DPAD_RIGHT]) != PRESSED
        && !dpad_toggle_)
    {
        dpad_toggle_ = true;
    }





    if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
    {
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
        joint_msg->header.frame_id = "arm_Link";
        joint_msg->header.stamp = nh_->now();

        joint_msg->joint_names.resize(6);
        joint_msg->joint_names = { "j1", "j2", "j3", "j4", "j5", "j6"};

        joint_msg->velocities.resize(6, 0.0);



        // DPAD RIGHT/LEFT used to switch between joints
        if (joy_msg->buttons[DPAD_RIGHT] == PRESSED && dpad_toggle_)
        {
            joint_num_++;
            if (joint_num_ > (NUM_JOINTS - 1)) joint_num_ = 0;

            RCLCPP_INFO(nh_->get_logger(), "Controlling J%d", (joint_num_ + 1));

            dpad_toggle_ = false;
            return;
        }
        else if (joy_msg->buttons[DPAD_LEFT] == PRESSED && dpad_toggle_)
        {
            joint_num_--;
            if (joint_num_ < 0) joint_num_ = (NUM_JOINTS - 1);

            RCLCPP_INFO(nh_->get_logger(), "Controlling J%d", (joint_num_ + 1));

            dpad_toggle_ = false;
            return;
        }




        // RIGHT/LEFT BUMPER used to move joint at fixed speed
        if (joy_msg->buttons[RIGHT_BUMPER] == PRESSED)
        {
            joint_msg->velocities[joint_num_] = joint_vel_cmd_;
            joint_pub_->publish(std::move(joint_msg));
        }
        if (joy_msg->buttons[LEFT_BUMPER] == PRESSED)
        {
            joint_msg->velocities[joint_num_] = -joint_vel_cmd_;
            joint_pub_->publish(std::move(joint_msg));
        }
        


        // RIGHT/LEFT TRIGGER(s) used to jog joint at variable speed
        if (joy_msg->axes[RIGHT_TRIGGER])
        {
            joint_msg->velocities[joint_num_] = (10 * joint_vel_cmd_) * (-joy_msg->axes[RIGHT_TRIGGER]);
            joint_pub_->publish(std::move(joint_msg));
        }
        if (joy_msg->axes[LEFT_TRIGGER])
        {
            joint_msg->velocities[joint_num_] = (10 * joint_vel_cmd_) * (joy_msg->axes[LEFT_TRIGGER]);
            joint_pub_->publish(std::move(joint_msg));
        }


        // joint_pub_->publish(std::move(joint_msg));

    }
    else if (servo_command_type_->command_type == moveit_msgs::srv::ServoCommandType::Request::TWIST)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.frame_id = command_frame_id_;
        twist_msg->header.stamp = nh_->now();

        // if (joy_msg->axes[RIGHT_STICK_X])
        // {
            // twist_msg->twist.linear.x = (-cartesian_step_size_ * 10) * joy_msg->axes[RIGHT_STICK_X];
            
            // twist_msg->twist.angular.x = joy_msg->axes[RIGHT_STICK_X];
            // twist_msg->twist.angular.z = 1;
            // twist_pub_->publish(std::move(twist_msg));
        // }
        
        // if (joy_msg->axes[RIGHT_STICK_Y])
        // {
            // twist_msg->twist.linear.y = (-cartesian_step_size_ * 10) * joy_msg->axes[RIGHT_STICK_Y];
            
            // twist_msg->twist.angular.y = joy_msg->axes[RIGHT_STICK_Y];
            // twist_msg->twist.angular.z = 1;
            // twist_pub_->publish(std::move(twist_msg));
        // }


        // RIGHT/LEFT BUMPER used to move up/down Z
        if (joy_msg->buttons[RIGHT_BUMPER] == PRESSED)
        {
            twist_msg->twist.linear.z = cartesian_step_size_;
            twist_pub_->publish(std::move(twist_msg));
            return;
        }
        if (joy_msg->buttons[LEFT_BUMPER] == PRESSED)
        {
            twist_msg->twist.linear.z = -cartesian_step_size_;
            twist_pub_->publish(std::move(twist_msg));
            return;
        }


        // TODO: DPAD RIGHT/LEFT used to switch between planes
        // if (joy_msg->buttons[DPAD_RIGHT] == PRESSED && dpad_toggle_)
        // {
        //     dpad_toggle_ = false;
        //     return;
        // }
        // else if (joy_msg->buttons[DPAD_LEFT] == PRESSED && dpad_toggle_)
        // {
        //     dpad_toggle_ = false;
        //     return;
        // }

        
        // LEFT STICK used to move end effector in xy plane
        if (joy_msg->axes[LEFT_STICK_X] || joy_msg->axes[LEFT_STICK_Y])
        {
            twist_msg->twist.linear.x = cartesian_step_size_ * joy_msg->axes[LEFT_STICK_X];
            twist_msg->twist.linear.y = cartesian_step_size_ * joy_msg->axes[LEFT_STICK_Y];
            twist_pub_->publish(std::move(twist_msg));
            return;
        }

    }

}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto game_controller = GameController();

    try
    {
        rclcpp::spin(game_controller.nh_);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

    rclcpp::shutdown();
    return 0;
}
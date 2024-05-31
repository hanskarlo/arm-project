/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
namespace
{
constexpr int8_t KEYCODE_RIGHT = 0x43;
constexpr int8_t KEYCODE_LEFT = 0x44;
constexpr int8_t KEYCODE_UP = 0x41;
constexpr int8_t KEYCODE_DOWN = 0x42;
constexpr int8_t KEYCODE_PERIOD = 0x2E;
constexpr int8_t KEYCODE_SEMICOLON = 0x3B;
constexpr int8_t KEYCODE_1 = 0x31;
constexpr int8_t KEYCODE_2 = 0x32;
constexpr int8_t KEYCODE_3 = 0x33;
constexpr int8_t KEYCODE_4 = 0x34;
constexpr int8_t KEYCODE_5 = 0x35;
constexpr int8_t KEYCODE_6 = 0x36;
constexpr int8_t KEYCODE_7 = 0x37;
constexpr int8_t KEYCODE_Q = 0x71;
constexpr int8_t KEYCODE_R = 0x72;
constexpr int8_t KEYCODE_J = 0x6A;
constexpr int8_t KEYCODE_T = 0x74;
constexpr int8_t KEYCODE_W = 0x77;
constexpr int8_t KEYCODE_E = 0x65;
constexpr int8_t KEYCODE_I = 0x69;
constexpr int8_t KEYCODE_O = 0x6F;
constexpr int8_t KEYCODE_P = 0x70;
}  // namespace

// Some constants used in the Servo Teleop demo
namespace
{
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string PLANNING_FRAME_ID = "arm_Link";
const std::string EE_FRAME_ID = "j6_Link";
}  // namespace

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : file_descriptor_(0)
  {
    // get the console in raw mode
    tcgetattr(file_descriptor_, &cooked_);
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(file_descriptor_, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(file_descriptor_, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(file_descriptor_, TCSANOW, &cooked_);
  }

private:
  int file_descriptor_;
  struct termios cooked_;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr service_node_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_start_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_servo_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr sw_hw_ctrl_mode_cli_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_servo_input_srv_;

  void pause_servo_cb_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request_;
  double joint_vel_cmd_;       // rad/s
  double cartesian_step_size_; // meters
  std::string command_frame_id_;
  bool input_paused_;
};

KeyboardServo::KeyboardServo() : 
  joint_vel_cmd_(0.1), 
  cartesian_step_size_(0.1), 
  command_frame_id_{ "arm_Link" },
  input_paused_(false)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");
  service_node_ = rclcpp::Node::make_shared("servo_keyboard_input_sn_");

  request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);

  reset_start_pos_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("arm/servo/reset", ROS_QUEUE_SIZE);

  // Client for switching input types
  switch_input_ = nh_->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/switch_command_type");

  // Client to un/pause servo node
  pause_servo_cli_ = service_node_->create_client<std_srvs::srv::SetBool>("servo_node/pause_servo");

  sw_hw_ctrl_mode_cli_ = service_node_->create_client<std_srvs::srv::SetBool>("arm_hw_node/toggle_servo_mode");

  // Service for pausing keyboard input and to pause servo node
  pause_servo_input_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
    "pause_servo_input", 
    std::bind(&KeyboardServo::pause_servo_cb_, this, std::placeholders::_1, std::placeholders::_2));
}

void KeyboardServo::pause_servo_cb_(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Suppress compiler warnings
  (void) request; 
  (void) response;

  RCLCPP_INFO(nh_->get_logger(), "Keyboard input paused.");
  input_paused_ = true;

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;

  auto future = pause_servo_cli_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(service_node_, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(nh_->get_logger(), "Called pause_servo service: %s", future.get()->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to call pause_servo service");
  }

  
  req->data = false;
  future = sw_hw_ctrl_mode_cli_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(service_node_, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(nh_->get_logger(), "Called pause_servo service: %s", future.get()->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to call pause_servo service");
  }

}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{ [this]() { return spin(); } }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("All commands are in the planning frame");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 1|2|3|4|5|6 keys to joint jog. 'r' to reverse the direction of jogging.");
  puts("Use 'j' to select joint jog. ");
  puts("Use 't' to select twist ");
  puts("Use 'w' and 'e' to switch between sending command in planning frame or end effector frame");
  puts("Use 'i' to increase the speed, 'o' to decrease the speed");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    if (input_paused_)
    {
      if (c == KEYCODE_P)
      {
        RCLCPP_INFO(nh_->get_logger(), "Keyboard input unpaused!");
        input_paused_ = false;

        // std_msgs::msg::Bool msg;
        // msg.data = true;
        // reset_start_pos_pub_->publish(msg);

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = false;

        auto future = pause_servo_cli_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(service_node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(nh_->get_logger(), "Called pause_servo service: %s", future.get()->message.c_str());
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "Failed to call pause_servo service");
        }

        req->data = true;
        future = sw_hw_ctrl_mode_cli_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(service_node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(nh_->get_logger(), "Called toggle_servo_mode service: %s", future.get()->message.c_str());
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "Failed to call pause_servo service");
        }

        continue;
      }
      else if (c == KEYCODE_Q)
      {
        RCLCPP_INFO(nh_->get_logger(), "quit");
        return 0;
      }
      else
      {
        RCLCPP_WARN(nh_->get_logger(), "Input paused, press 'p' to unpause");
        continue;
      }
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);
    // RCLCPP_INFO(nh_->get_logger(), "value: 0x%02X\n", c);

    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    joint_msg->joint_names.resize(6);
    joint_msg->joint_names = { "j1", "j2", "j3", "j4",
                               "j5", "j6"};

    joint_msg->velocities.resize(6);
    std::fill(joint_msg->velocities.begin(), joint_msg->velocities.end(), 0.0);
    // Use read key-press
    switch (c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = cartesian_step_size_;
        publish_twist = true;
        break;
      case KEYCODE_1:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg->velocities[0] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_2:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg->velocities[1] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_3:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg->velocities[2] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_4:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg->velocities[3] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_5:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg->velocities[4] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_6:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg->velocities[5] = joint_vel_cmd_;
        publish_joint = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "r");
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_J:
        RCLCPP_DEBUG(nh_->get_logger(), "j");
        request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
        if (switch_input_->wait_for_service(std::chrono::seconds(1)))
        {
          auto result = switch_input_->async_send_request(request_);
          if (result.get()->success)
          {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Switched to input type: JointJog");
          }
          else
          {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not switch input to: JointJog");
          }
        }
        break;
      case KEYCODE_T:
        RCLCPP_DEBUG(nh_->get_logger(), "t");
        request_->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
        if (switch_input_->wait_for_service(std::chrono::seconds(1)))
        {
          auto result = switch_input_->async_send_request(request_);
          if (result.get()->success)
          {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Switched to input type: Twist");
          }
          else
          {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not switch input to: Twist");
          }
        }
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "w");
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Command frame set to: " << PLANNING_FRAME_ID);
        command_frame_id_ = PLANNING_FRAME_ID;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "e");
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Command frame set to: " << EE_FRAME_ID);
        command_frame_id_ = EE_FRAME_ID;
        break;
      case KEYCODE_I:
        if (request_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
        {
          if (joint_vel_cmd_ > 0)
            joint_vel_cmd_ += 0.1;
          else
            joint_vel_cmd_ -= 0.1;

          RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint velocity increased: " << joint_vel_cmd_ << "rad/s");
        }
        else if (request_->command_type == moveit_msgs::srv::ServoCommandType::Request::TWIST)
        {
          cartesian_step_size_ += 0.01; //cm
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Cartesian step size increased: " << cartesian_step_size_ << "m");
        }
        break;
      case KEYCODE_O:
        if (request_->command_type == moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG)
        {
          if (joint_vel_cmd_ > 0)
            joint_vel_cmd_ -= 0.1;
          else
            joint_vel_cmd_ += 0.1;

          RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint velocity decreased: " << joint_vel_cmd_ << "rad/s");
        }
        else if (request_->command_type == moveit_msgs::srv::ServoCommandType::Request::TWIST)
        {
          cartesian_step_size_ -= 0.01; //cm
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Cartesian step size decreased: " << cartesian_step_size_ << "m");
        }
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = command_frame_id_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = PLANNING_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}
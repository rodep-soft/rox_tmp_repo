#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "custom_interfaces/msg/cmd_dpad.hpp"
#include "custom_interfaces/msg/mode_status.hpp"
#include "custom_interfaces/srv/set_mode.hpp"

enum class RobotMode : uint8_t {
  DRIVE = 0,
  STOP = 1
};

class JoyDriverNode : public rclcpp::Node {
 public:
  JoyDriverNode() : Node("joy_driver_node"), current_mode_(RobotMode::DRIVE) {
    // Declare and get parameters for velocity scaling and axis mapping
    declare_parameters();
    get_parameters();

    // Create subscription to the /joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));

    // Create publisher for the /cmd_vel topic
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    cmd_dpad_publisher_ = this->create_publisher<custom_interfaces::msg::CmdDpad>("cmd_dpad", 10);

    // Create mode status publisher
    mode_status_publisher_ = this->create_publisher<custom_interfaces::msg::ModeStatus>("mode_status", 10);

    // Create mode service
    mode_service_ = this->create_service<custom_interfaces::srv::SetMode>(
        "set_mode", std::bind(&JoyDriverNode::set_mode_callback, this, 
        std::placeholders::_1, std::placeholders::_2));

    // Timer to publish mode status periodically
    mode_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoyDriverNode::publish_mode_status, this));

    RCLCPP_INFO(this->get_logger(), "Joy driver node started.");
  }

 private:
  void declare_parameters() {
    this->declare_parameter<double>("linear_x_scale", 1.0);
    this->declare_parameter<double>("linear_y_scale", 1.0);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<int>("linear_x_axis", 1);  // Vertical movement
    this->declare_parameter<int>("linear_y_axis", 0);  // Horizontal movement
    this->declare_parameter<int>("angular_axis", 3);
  }

  void get_parameters() {
    linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
    linear_y_scale_ = this->get_parameter("linear_y_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    linear_x_axis_ = this->get_parameter("linear_x_axis").as_int();
    linear_y_axis_ = this->get_parameter("linear_y_axis").as_int();
    angular_axis_ = this->get_parameter("angular_axis").as_int();
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Ensure the message has enough axes to prevent a crash
    if (msg->axes.size() <=
        static_cast<size_t>(std::max({linear_x_axis_, linear_y_axis_, angular_axis_}))) {
      RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient axes.");
      return;
    }

    // Check for mode switching via buttons (example: button 0 for DRIVE, button 1 for STOP)
    if (msg->buttons.size() > 1) {
      if (msg->buttons[0] == 1 && current_mode_ != RobotMode::DRIVE) {
        set_mode(RobotMode::DRIVE);
        RCLCPP_INFO(this->get_logger(), "Mode switched to DRIVE via joystick");
      } else if (msg->buttons[1] == 1 && current_mode_ != RobotMode::STOP) {
        set_mode(RobotMode::STOP);
        RCLCPP_INFO(this->get_logger(), "Mode switched to STOP via joystick");
      }
    }

    // Map joystick axes to velocity commands
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = msg->axes[linear_x_axis_] * linear_x_scale_;
    twist_msg->linear.y = msg->axes[linear_y_axis_] * linear_y_scale_;
    twist_msg->angular.z = msg->axes[angular_axis_] * angular_scale_;

    // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f,
    // angular.z=%.2f",
    //             twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);

    // Publish the velocity command

    auto dpad_msg = std::make_unique<custom_interfaces::msg::CmdDpad>();
    dpad_msg->up = msg->buttons[11];
    dpad_msg->down = msg->buttons[12];
    dpad_msg->left = msg->buttons[13];
    dpad_msg->right = msg->buttons[14];

    cmd_vel_publisher_->publish(std::move(twist_msg));
    cmd_dpad_publisher_->publish(std::move(dpad_msg));
  }

  void set_mode_callback(const std::shared_ptr<custom_interfaces::srv::SetMode::Request> request,
                        std::shared_ptr<custom_interfaces::srv::SetMode::Response> response) {
    RobotMode new_mode = static_cast<RobotMode>(request->mode);
    
    if (new_mode == RobotMode::DRIVE || new_mode == RobotMode::STOP) {
      set_mode(new_mode);
      response->success = true;
      response->message = (new_mode == RobotMode::DRIVE) ? "Mode set to DRIVE" : "Mode set to STOP";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "Invalid mode";
      RCLCPP_WARN(this->get_logger(), "Invalid mode requested: %d", request->mode);
    }
  }

  void set_mode(RobotMode mode) {
    current_mode_ = mode;
  }

  void publish_mode_status() {
    auto msg = std::make_unique<custom_interfaces::msg::ModeStatus>();
    msg->current_mode = static_cast<uint8_t>(current_mode_);
    mode_status_publisher_->publish(std::move(msg));
  }

  // ROS 2 components
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::CmdDpad>::SharedPtr cmd_dpad_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::ModeStatus>::SharedPtr mode_status_publisher_;
  rclcpp::Service<custom_interfaces::srv::SetMode>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr mode_timer_;

  // Mode state
  RobotMode current_mode_;

  // Parameters
  double linear_x_scale_;
  double linear_y_scale_;
  double angular_scale_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_axis_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriverNode>());
  rclcpp::shutdown();
  return 0;
}

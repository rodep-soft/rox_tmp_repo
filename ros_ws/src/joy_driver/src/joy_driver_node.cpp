#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "custom_interfaces/msg/cmd_dpad.hpp"
#include <memory>
#include <algorithm>

class JoyDriverNode : public rclcpp::Node
{
public:
  JoyDriverNode() : Node("joy_driver_node")
  {
    // Declare and get parameters for velocity scaling and axis mapping
    declare_parameters();
    get_parameters();

    // Create subscription to the /joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));
    
    // Create publisher for the /cmd_vel topic
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
   
    cmd_dpad_publisher_ = this->create_publisher<custom_interfaces::msg::CmdDpad>("cmd_dpad", 10);

    RCLCPP_INFO(this->get_logger(), "Joy driver node started.");
  }

private:
  void declare_parameters()
  {
    this->declare_parameter<double>("linear_x_scale", 1.0);
    this->declare_parameter<double>("linear_y_scale", 1.0);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<int>("linear_x_axis", 1); // Vertical movement
    this->declare_parameter<int>("linear_y_axis", 0); // Horizontal movement
    this->declare_parameter<int>("angular_axis", 3);
  }

  void get_parameters()
  {
    linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
    linear_y_scale_ = this->get_parameter("linear_y_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    linear_x_axis_ = this->get_parameter("linear_x_axis").as_int();
    linear_y_axis_ = this->get_parameter("linear_y_axis").as_int();
    angular_axis_ = this->get_parameter("angular_axis").as_int();
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ensure the message has enough axes to prevent a crash
    if (msg->axes.size() <= static_cast<size_t>(std::max({linear_x_axis_, linear_y_axis_, angular_axis_}))) {
      RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient axes.");
      return;
    }

    // Map joystick axes to velocity commands
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = msg->axes[linear_x_axis_] * linear_x_scale_;
    twist_msg->linear.y = msg->axes[linear_y_axis_] * linear_y_scale_;
    twist_msg->angular.z = msg->axes[angular_axis_] * angular_scale_;

    // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
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

  // ROS 2 components
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::CmdDpad>::SharedPtr cmd_dpad_publisher_;


  // Parameters
  double linear_x_scale_;
  double linear_y_scale_;
  double angular_scale_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_axis_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriverNode>());
  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyDriverNode : public rclcpp::Node
{
public:
  JoyDriverNode() : Node("joy_driver_node")
  {
    // Create subscription to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));
    
    // Create publisher for cmd_vel
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Parameters for velocity scaling
    this->declare_parameter("linear_scale", 1.0);
    this->declare_parameter("angular_scale", 1.0);
    this->declare_parameter("linear_axis", 1);  // Typically left stick vertical
    this->declare_parameter("angular_axis", 0); // Typically left stick horizontal
    
    RCLCPP_INFO(this->get_logger(), "Joy driver node started, waiting for joy messages...");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Get parameters
    double linear_scale = this->get_parameter("linear_scale").as_double();
    double angular_scale = this->get_parameter("angular_scale").as_double();
    int linear_axis = this->get_parameter("linear_axis").as_int();
    int angular_axis = this->get_parameter("angular_axis").as_int();
    
    // Create cmd_vel message
    auto twist_msg = geometry_msgs::msg::Twist();
    
    // Check if axes exist
    if (msg->axes.size() > static_cast<size_t>(linear_axis) && 
        msg->axes.size() > static_cast<size_t>(angular_axis)) {
      
      // Map joystick axes to velocity
      twist_msg.linear.x = msg->axes[linear_axis] * linear_scale;
      twist_msg.angular.z = msg->axes[angular_axis] * angular_scale;
      
      // Publish cmd_vel
      cmd_vel_publisher_->publish(twist_msg);
      
      RCLCPP_INFO(this->get_logger(), 
                  "Published cmd_vel: linear.x=%.3f, angular.z=%.3f", 
                  twist_msg.linear.x, twist_msg.angular.z);
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "Not enough axes in joy message. Expected at least %d axes, got %zu",
                  std::max(linear_axis, angular_axis) + 1, msg->axes.size());
    }
    
    // Debug: Print joy message info
    RCLCPP_DEBUG(this->get_logger(), "Received joy message:");
    RCLCPP_DEBUG(this->get_logger(), "  Axes count: %zu", msg->axes.size());
    RCLCPP_DEBUG(this->get_logger(), "  Buttons count: %zu", msg->buttons.size());
    
    // Print axes values (debug)
    for (size_t i = 0; i < msg->axes.size(); ++i) {
      RCLCPP_DEBUG(this->get_logger(), "  Axis %zu: %.3f", i, msg->axes[i]);
    }
    
    // Print button values (debug)
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i]) {
        RCLCPP_DEBUG(this->get_logger(), "  Button %zu: PRESSED", i);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriverNode>());
  rclcpp::shutdown();
  return 0;
}
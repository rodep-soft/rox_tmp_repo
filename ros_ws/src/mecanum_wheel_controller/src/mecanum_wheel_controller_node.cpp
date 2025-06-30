#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MecanumWheelControllerNode : public rclcpp::Node
{
public:
  MecanumWheelControllerNode() : Node("mecanum_wheel_controller_node")
  {
    // Create subscription to cmd_vel topic
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MecanumWheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    
    // Parameters for mecanum wheel configuration
    this->declare_parameter("wheel_radius", 0.05);      // Wheel radius in meters
    this->declare_parameter("wheel_base_x", 0.3);       // Distance between front and rear wheels
    this->declare_parameter("wheel_base_y", 0.3);       // Distance between left and right wheels
    
    RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node started, waiting for cmd_vel messages...");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Get parameters
    double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_base_x = this->get_parameter("wheel_base_x").as_double();
    double wheel_base_y = this->get_parameter("wheel_base_y").as_double();
    
    // Extract velocity components
    double vx = msg->linear.x;   // Forward/backward velocity
    double vy = msg->linear.y;   // Left/right strafe velocity
    double wz = msg->angular.z;  // Rotational velocity
    
    // Mecanum wheel kinematics
    // For a mecanum wheel robot with wheels arranged as:
    //   FL(+)  FR(-)
    //   BL(-)  BR(+)
    // Where (+) means forward rolling gives positive robot motion
    
    double lx = wheel_base_x / 2.0;  // Half distance between front and rear
    double ly = wheel_base_y / 2.0;  // Half distance between left and right
    
    // Calculate individual wheel velocities (rad/s)
    double front_left_vel  = (vx - vy - (lx + ly) * wz) / wheel_radius;
    double front_right_vel = (vx + vy + (lx + ly) * wz) / wheel_radius;
    double back_left_vel   = (vx + vy - (lx + ly) * wz) / wheel_radius;
    double back_right_vel  = (vx - vy + (lx + ly) * wz) / wheel_radius;
    
    // Log the received cmd_vel and calculated wheel velocities
    RCLCPP_INFO(this->get_logger(), 
                "Received cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f", 
                vx, vy, wz);
    
    RCLCPP_INFO(this->get_logger(), 
                "Wheel velocities (rad/s): FL=%.3f, FR=%.3f, BL=%.3f, BR=%.3f",
                front_left_vel, front_right_vel, back_left_vel, back_right_vel);
    
    // TODO: Here you would send these velocities to your motor controllers
    // For example, publish to motor command topics or send via serial/CAN
    
    // Example of what you might do:
    // publish_motor_commands(front_left_vel, front_right_vel, back_left_vel, back_right_vel);
  }
  
  // TODO: Add methods to interface with your motor controllers
  // void publish_motor_commands(double fl, double fr, double bl, double br) {
  //   // Send commands to motors via your preferred interface
  // }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}

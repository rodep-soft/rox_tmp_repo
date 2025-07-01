#include <memory>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "mecanum_wheel_controller/ddsm_ctrl.hpp"

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
    this->declare_parameter("serial_port", "/dev/ttyUSB0");  // Serial port for DDSM motors
    this->declare_parameter("baud_rate", 115200);       // Baud rate for serial communication
    this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2, 3, 4});  // Motor IDs [FL, FR, BL, BR]
    
    // Initialize DDSM controllers for each motor
    for (int i = 0; i < 4; i++) {
      ddsm_controllers_[i] = std::make_unique<DDSM_CTRL>();
    }
    
    // Initialize serial connection
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    
    bool init_success = true;
    for (int i = 0; i < 4; i++) {
      if (!ddsm_controllers_[i]->init(port, baud)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize DDSM controller %d", i);
        init_success = false;
      }
    }
    
    if (init_success) {
      RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node started successfully!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Mecanum wheel controller node started with some initialization errors");
    }
    
    RCLCPP_INFO(this->get_logger(), "Waiting for cmd_vel messages...");
  }
  
  ~MecanumWheelControllerNode() {
    // Stop all motors on shutdown
    stop_all_motors();
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
    
    // Send commands to DDSM motors
    send_motor_commands(front_left_vel, front_right_vel, back_left_vel, back_right_vel);
  }

  void send_motor_commands(double fl, double fr, double bl, double br) {
    auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    
    // Convert rad/s to motor commands (assuming speed mode)
    // Scale factor may need adjustment based on your motor specifications
    const double scale_factor = 30.0; // Adjust this based on your motors
    
    int fl_cmd = static_cast<int>(fl * scale_factor);
    int fr_cmd = static_cast<int>(fr * scale_factor);
    int bl_cmd = static_cast<int>(bl * scale_factor);
    int br_cmd = static_cast<int>(br * scale_factor);
    
    // Clamp commands to motor limits
    fl_cmd = std::max(-200, std::min(200, fl_cmd));
    fr_cmd = std::max(-200, std::min(200, fr_cmd));
    bl_cmd = std::max(-200, std::min(200, bl_cmd));
    br_cmd = std::max(-200, std::min(200, br_cmd));
    
    try {
      // Send commands to each motor (FL, FR, BL, BR)
      if (motor_ids.size() >= 4) {
        ddsm_controllers_[0]->ddsm_ctrl(motor_ids[0], fl_cmd, 1); // Front Left
        ddsm_controllers_[1]->ddsm_ctrl(motor_ids[1], fr_cmd, 1); // Front Right
        ddsm_controllers_[2]->ddsm_ctrl(motor_ids[2], bl_cmd, 1); // Back Left
        ddsm_controllers_[3]->ddsm_ctrl(motor_ids[3], br_cmd, 1); // Back Right
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending motor commands: %s", e.what());
    }
  }
  
  void stop_all_motors() {
    auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    
    try {
      for (size_t i = 0; i < std::min(motor_ids.size(), static_cast<size_t>(4)); i++) {
        ddsm_controllers_[i]->ddsm_stop(motor_ids[i]);
      }
      RCLCPP_INFO(this->get_logger(), "All motors stopped");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error stopping motors: %s", e.what());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  std::unique_ptr<DDSM_CTRL> ddsm_controllers_[4]; // FL, FR, BL, BR
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}

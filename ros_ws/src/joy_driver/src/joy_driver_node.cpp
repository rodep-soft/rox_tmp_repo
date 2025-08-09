#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "custom_interfaces/msg/cmd_dpad.hpp"
#include "custom_interfaces/msg/upper_motor.hpp"

class JoyDriverNode : public rclcpp::Node {
 public:
  JoyDriverNode() : Node("joy_driver_node") {
    // Declare and get parameters for velocity scaling and axis mapping
    declare_parameters();
    get_parameters();

    // Create subscription to the /joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", best_effort_qos,
        std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));

    rpy_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/imu/rpy", best_effort_qos,
        std::bind(&JoyDriverNode::rpy_callback, this, std::placeholders::_1));

    // Create publisher for the /cmd_vel topic
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", best_effort_qos);

    brake_client_ = this->create_client<std_srvs::srv::SetBool>("/brake");

    upper_publisher_ =
        this->create_publisher<custom_interfaces::msg::UpperMotor>("/upper_motor", 3);

    linetrace_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/is_linetrace", 10);

    mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mode", 10);

    RCLCPP_INFO(this->get_logger(), "Joy driver node started.");
  }

 private:
  // Define the mode of operation for the robot
  enum class Mode { STOP, JOY, DPAD, LINETRACE };

  Mode mode_ = Mode::STOP;

  // Previous button states for toggle functionality
  bool prev_linetrace_buttons_ = false;

  bool prev_throwing_on = false;
  bool prev_ejection_on = false;
  bool prev_elevation_on = false;

  // static float applyDeadzone(double val, double threshold) {
  //   return (std::abs(val) < threshold) ? 0.0f : val;
  // }

  // share buttons[4]
  // option buttons[6]

  // ROS2パラメータ
  // config/mecanum.yamlを変更することで再ビルドかけずにパラメータの変更が可能
  void declare_parameters() {
    this->declare_parameter<double>("linear_x_scale", 1.0);
    this->declare_parameter<double>("linear_y_scale", 1.0);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<int>("linear_x_axis", 1);  // Vertical movement
    this->declare_parameter<int>("linear_y_axis", 0);  // Horizontal movement
    this->declare_parameter<int>("angular_axis", 3);

    this->declare_parameter<double>("Kp", 1.0);  // DPADモードのずれを補正する比例ゲイン
  }

  // パラメータを取得する関数
  void get_parameters() {
    linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
    linear_y_scale_ = this->get_parameter("linear_y_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    linear_x_axis_ = this->get_parameter("linear_x_axis").as_int();
    linear_y_axis_ = this->get_parameter("linear_y_axis").as_int();
    angular_axis_ = this->get_parameter("angular_axis").as_int();

    Kp = this->get_parameter("Kp").as_double();  // 比例ゲイン
  }

  // ----- メインのコールバック関数 -----
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

    auto linetrace_msg = std::make_unique<std_msgs::msg::Bool>();

    auto mode_msg = std::make_unique<std_msgs::msg::String>();

    // だめぽ
    // linetrace_msg->data = false;

    // Ensure the message has enough axes to prevent a crash
    if (msg->axes.size() <=
        static_cast<size_t>(std::max({linear_x_axis_, linear_y_axis_, angular_axis_}))) {
      RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient axes.");
      return;
    }

    // buttons[11] == 1 && buttons[12] == 1
    // linetrace_msg->data = 1
    //

    // Mode switching logic
    bool current_linetrace_buttons = (msg->buttons[7] == 1 && msg->buttons[8] == 1);

    // ライントレースモードかどうかを変える
    if (current_linetrace_buttons && !prev_linetrace_buttons_) {
      // Toggle LINETRACE mode only on button press (not hold)
      if (mode_ == Mode::LINETRACE) {
        mode_ = Mode::JOY;
        RCLCPP_INFO(this->get_logger(), "Mode: JOY (from LINETRACE)");
      } else {
        mode_ = Mode::LINETRACE;
        RCLCPP_INFO(this->get_logger(), "Mode: LINETRACE");
      }
    }

    // もしライントレースモードでなければ、他のモードに切り替えることができる
    if (mode_ != Mode::LINETRACE) {
      if (msg->buttons[4] == 1 && mode_ != Mode::JOY) {
        mode_ = Mode::JOY;
        RCLCPP_INFO(this->get_logger(), "Mode: JOY");
      } else if (msg->buttons[5] == 1 && mode_ != Mode::STOP) {
        mode_ = Mode::STOP;
        RCLCPP_INFO(this->get_logger(), "Mode: STOP");
      } else if (msg->buttons[6] == 1 && mode_ != Mode::DPAD) {
        mode_ = Mode::DPAD;
        init_yaw_ = yaw_;
        RCLCPP_INFO(this->get_logger(), "Mode: DPAD");
      }
    }

    // Update previous button state
    prev_linetrace_buttons_ = current_linetrace_buttons;

    bool l2_pressed = msg->axes[4] < TRIGGER_THRESHOLD;  // L2 trigger
    bool r2_pressed = msg->axes[5] < TRIGGER_THRESHOLD;  // R2 trigger

    // Initialize the twist message
    twist_msg->linear.x = 0.0;
    twist_msg->linear.y = 0.0;
    twist_msg->angular.z = 0.0;

    // if (r2_pressed && !l2_pressed) {
    //       // R2: rotate left
    //       twist_msg->angular.z = -(msg->axes[4] - 1) / 2.0;
    //     } else if (l2_pressed && !r2_pressed) {
    //       // L2: rotate right
    //       twist_msg->angular.z = (msg->axes[5] - 1) / 2.0;
    //     }

    // switch (mode_){
    //   case Mode::JOY:
    //     if(!l2_pressed && !r2_pressed) {
    //       twist_msg->linear.x = msg->axes[linear_x_axis_] * linear_x_scale_;
    //       twist_msg->linear.y = msg->axes[linear_y_axis_] * linear_y_scale_;
    //       twist_msg->angular.z = 0.0;
    //     } else {
    //       set_angular_velocity(msg, twist_msg);
    //     }
    //     break;
    //   case Mode::DPAD:
    //     if(!l2_pressed && !r2_pressed) {
    //       twist_msg->linear.x = (msg->buttons[11] - msg->buttons[12]) * linear_x_scale_ / 2.0;
    //       twist_msg->linear.y = (msg->buttons[13] - msg->buttons[14]) * linear_y_scale_ / 2.0;
    //       twist_msg->angular.z = 0.0;  // No angular movement in DPAD mode
    //     } else {
    //       set_angular_velocity(msg, twist_msg);
    //     }
    //     break;
    //   case Mode::STOP:
    //     twist_msg->linear.x = 0.0;
    //     twist_msg->linear.y = 0.0;
    //     twist_msg->angular.z = 0.0;  // No movement in STOP mode
    //     break;
    //   default:
    // }

    double error = yaw_ - init_yaw_;

    switch (mode_) {
      case Mode::STOP:
        twist_msg->linear.x = 0.0;
        twist_msg->linear.y = 0.0;
        twist_msg->angular.z = 0.0;
        break;
      case Mode::JOY:
        if (!l2_pressed && !r2_pressed) {  // When neither L2 nor R2 are pressed
          twist_msg->linear.x = msg->axes[linear_x_axis_] * linear_x_scale_;
          twist_msg->linear.y = msg->axes[linear_y_axis_] * linear_y_scale_;
          twist_msg->angular.z = 0.0;
        } else {  // When either L2 or R2 is pressed
          twist_msg->angular.z = get_angular_velocity(msg);
        }
        break;
      case Mode::DPAD:
        if (!l2_pressed && !r2_pressed) {
          twist_msg->linear.x = (msg->buttons[11] - msg->buttons[12]) * linear_x_scale_ / 2.0;
          twist_msg->linear.y = (msg->buttons[13] - msg->buttons[14]) * linear_y_scale_ / 2.0;
          // twist_msg->angular.z = 0.0;
          // 要検討
          if (error < 0.05 || error > -0.05) {
            twist_msg->angular.z = 0.0;
          } else {
            twist_msg->angular.z = std::clamp(error * Kp, -0.5, 0.5);
          }
        } else {
          twist_msg->angular.z = get_angular_velocity(msg);
        }
        break;
      case Mode::LINETRACE:
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown mode: %d", static_cast<int>(mode_));
    }

    // cmd_velのpublish
    if (mode_ != Mode::LINETRACE) {
      cmd_vel_publisher_->publish(std::move(twist_msg));
    }

    // Map joystick axes to velocity commands
    // auto twist_msg = set_velocity(msg);
    // auto twist_msg = set_angular_velocity(msg);

    // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f,
    // angular.z=%.2f",
    //             twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);

    // Publish the velocity command

    // auto dpad_msg = std::make_unique<custom_interfaces::msg::CmdDpad>();
    // dpad_msg->up = msg->buttons[11];
    // dpad_msg->down = msg->buttons[12];
    // dpad_msg->left = msg->buttons[13];
    // dpad_msg->right = msg->buttons[14];

    auto upper_msg = std::make_unique<custom_interfaces::msg::UpperMotor>();
    // UpperMotor.msgのフィールド設定

    // システム準備状態（例：SELECT + STARTボタン同時押し）
    // これはどうするか迷い中
    upper_msg->is_system_ready = (msg->buttons[4] == 1 && msg->buttons[6] == 1);

    // square button (射出)
    if (msg->buttons[2] == 1) {
      upper_msg->is_throwing_on = true;
    } else {
      upper_msg->is_throwing_on = false;
    }

    // circle button (押出)
    if (msg->buttons[1] == 1) {
      upper_msg->is_ejection_on = true;
    } else {
      upper_msg->is_ejection_on = false;
    }

    // 昇降制御（方向パッド）
    if (msg->buttons[3] == 1) {        // triangle
      upper_msg->elevation_mode = 1;    // 上昇
    } else if (msg->buttons[12] == 1) { // x
      upper_msg->elevation_mode = 0;    // 下降
    } else {
      upper_msg->elevation_mode = 2;    // 停止
    }



    //  一旦廃止
    // // Check for state changes
    // if (msg->buttons[1] == 1 && prev_throwing_on == false) {
    //   prev_throwing_on = true;
    //   RCLCPP_INFO(this->get_logger(), "Throwing on");
    // } else if (msg->buttons[3] == 1 && prev_throwing_on == true) {
    //   prev_throwing_on = false;
    //   RCLCPP_INFO(this->get_logger(), "Throwing off");
    // } else if (msg->buttons[0] == 1 && prev_ejection_on == false) {
    //   prev_ejection_on = true;
    //   RCLCPP_INFO(this->get_logger(), "Ejection on");
    // } else if (msg->buttons[2] == 1 && prev_ejection_on == true) {
    //   prev_ejection_on = false;
    //   RCLCPP_INFO(this->get_logger(), "Ejection off");
    // } else if (msg->buttons[10] == 1 && prev_elevation_on == false) {
    //   prev_elevation_on = true;
    //   RCLCPP_INFO(this->get_logger(), "Elevation on");
    // } else if (msg->buttons[9] == 1 && prev_elevation_on == true) {
    //   prev_elevation_on = false;
    //   RCLCPP_INFO(this->get_logger(), "Elevation off");
    // }

    // // Always set current state to the message
    // upper_msg->is_throwing_on = prev_throwing_on;
    // upper_msg->is_ejection_on = prev_ejection_on;
    // upper_msg->is_elevation_on = prev_elevation_on;

    // Always publish current state
    upper_publisher_->publish(std::move(upper_msg));
    // cmd_dpad_publisher_->publish(std::move(dpad_msg));

    if (mode_ == Mode::LINETRACE) {
      linetrace_msg->data = true;
    } else {
      linetrace_msg->data = false;
    }

    linetrace_publisher_->publish(std::move(linetrace_msg));

    mode_msg->data = mode_to_string(mode_);
    mode_publisher_->publish(std::move(mode_msg));
  }

  // モードを文字列に変換する関数
  std::string mode_to_string(Mode mode) {
    switch (mode) {
      case Mode::STOP:
        return "STOP";
      case Mode::JOY:
        return "JOY";
      case Mode::DPAD:
        return "DPAD";
      case Mode::LINETRACE:
        return "LINETRACE";
      default:
        return "UNKNOWN";
    }
  }

  // オイラー角を受信するためのコールバック関数
  void rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    roll_ = msg->x;
    pitch_ = msg->y;
    yaw_ = msg->z;  // current yaw value
  }

  // void set_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg,
  //                          std::unique_ptr<geometry_msgs::msg::Twist>& twist_msg) {
  //   if(msg->axes[4] < TRIGGER_THRESHOLD && msg->axes[5] >= TRIGGER_THRESHOLD) {
  //     // R2: rotate right
  //     twist_msg->angular.z = -(msg->axes[4] - 1) / 2.0;
  //   } else if(msg->axes[5] < TRIGGER_THRESHOLD && msg->axes[4] >= TRIGGER_THRESHOLD) {
  //     // L2: rotate left
  //     twist_msg->angular.z = (msg->axes[5] - 1) / 2.0;
  //   } else {
  //     twist_msg->angular.z = 0.0;
  //   }
  // }

  double get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg) {
    if (mode_ == Mode::DPAD) {
      // 基準値を変更する
      init_yaw_ = yaw_;
    }

    if (msg->axes[4] < TRIGGER_THRESHOLD && msg->axes[5] >= TRIGGER_THRESHOLD) {
      // R2: rotate right
      return -(msg->axes[4] - 1) / 2.0;
    } else if (msg->axes[5] < TRIGGER_THRESHOLD && msg->axes[4] >= TRIGGER_THRESHOLD) {
      // L2: rotate left
      return (msg->axes[5] - 1) / 2.0;
    } else {
      return 0.0;
    }
  }

  const double TRIGGER_THRESHOLD = 0.95;

  // ROS 2 components
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
      rpy_subscription_;  // オイラー角を受信するためのsubscription

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::UpperMotor>::SharedPtr upper_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr linetrace_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr brake_client_;

  // Parameters
  double linear_x_scale_;
  double linear_y_scale_;
  double angular_scale_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_axis_;

  // PID制御のゲイン
  double Kp;  // 比例ゲイン

  // オイラー角
  double pitch_ = 0.0;
  double roll_ = 0.0;
  double yaw_ = 0.0;

  // 初期化時のyaw値
  // これはロボットの初期姿勢を基準にするための値
  // これを用いてDPADモードでの直線移動での回転方向のズレを補正する
  // L2, R2を押した時も値を更新する
  double init_yaw_ = 0.0;

  const rclcpp::QoS reliable_qos = rclcpp::QoS(1).reliable();
  const rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDriverNode>());
  rclcpp::shutdown();
  return 0;
}



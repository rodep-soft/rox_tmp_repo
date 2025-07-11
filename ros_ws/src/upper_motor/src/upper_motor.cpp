#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/cmd_dpad.hpp"
#include <memory>
#include <functional>



class UpperMotor {
  public:
    UpperMotor() {
      dpad_subscription_ = this->create_subscription<custom_interfaces::msg::CmdDpad>(
        "cmd_dpad", 10, 
        std::bind(&UpperMotor::dpad_callback, this, std::placeholders::_1)
      );

    }


  private:
    rclcpp::Subscription<custom_interfaces::msg::CmdDpad>::SharedPtr dpad_subscription_;

    void dpad_callback(const custom_interfaces::msg::CmdDpad::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received DPad command: %d", msg->up);
    }

};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<UpperMotor>());
  rclcpp::shutdown();
  return 0;
}

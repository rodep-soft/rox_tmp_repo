#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "joy_driver/joy_driver_node.hpp"

class JoyDriverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<JoyDriverNode>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<JoyDriverNode> node_;
};

// Test deadzone function
TEST_F(JoyDriverTest, TestApplyDeadzone) {
  // Test values within deadzone should return 0
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.1, 0.15), 0.0);
  EXPECT_EQ(JoyDriverNode::applyDeadzone(-0.1, 0.15), 0.0);
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.14, 0.15), 0.0);

  // Test values outside deadzone should return original value
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.2, 0.15), 0.2);
  EXPECT_EQ(JoyDriverNode::applyDeadzone(-0.3, 0.15), -0.3);
  EXPECT_EQ(JoyDriverNode::applyDeadzone(1.0, 0.15), 1.0);
}

// Test mode string conversion
TEST_F(JoyDriverTest, TestModeToString) {
  EXPECT_EQ(node_->mode_to_string(JoyDriverNode::Mode::STOP), "STOP");
  EXPECT_EQ(node_->mode_to_string(JoyDriverNode::Mode::JOY), "JOY");
  EXPECT_EQ(node_->mode_to_string(JoyDriverNode::Mode::DPAD), "DPAD");
  EXPECT_EQ(node_->mode_to_string(JoyDriverNode::Mode::LINETRACE), "LINETRACE");
}

// Test joystick message processing
TEST_F(JoyDriverTest, TestJoyMessageProcessing) {
  // Create a test joystick message
  auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();

  // Initialize with proper size
  joy_msg->axes.resize(6, 0.0);
  joy_msg->buttons.resize(16, 0);

  // Set some test values
  joy_msg->axes[0] = 0.5;  // linear_y_axis
  joy_msg->axes[1] = 0.3;  // linear_x_axis
  joy_msg->axes[2] = 0.1;  // angular_axis (should be filtered by deadzone)

  // Test that the node can process this message without crashing
  EXPECT_NO_THROW(node_->joy_callback(joy_msg));
}

// Test angular velocity calculation
TEST_F(JoyDriverTest, TestAngularVelocityCalculation) {
  auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
  joy_msg->axes.resize(6, 0.0);
  joy_msg->buttons.resize(16, 0);

  // Test L2 trigger pressed (left rotation)
  joy_msg->axes[4] = -1.0;  // L2 fully pressed
  joy_msg->axes[5] = 1.0;   // R2 not pressed

  double angular_vel = node_->get_angular_velocity(joy_msg);
  EXPECT_GT(angular_vel, 0.0);  // Should be positive (left rotation)

  // Test R2 trigger pressed (right rotation)
  joy_msg->axes[4] = 1.0;   // L2 not pressed
  joy_msg->axes[5] = -1.0;  // R2 fully pressed

  angular_vel = node_->get_angular_velocity(joy_msg);
  EXPECT_LT(angular_vel, 0.0);  // Should be negative (right rotation)

  // Test no triggers pressed
  joy_msg->axes[4] = 1.0;  // L2 not pressed
  joy_msg->axes[5] = 1.0;  // R2 not pressed

  angular_vel = node_->get_angular_velocity(joy_msg);
  EXPECT_EQ(angular_vel, 0.0);  // Should be zero
}

// Test deadzone filtering for different scenarios
TEST_F(JoyDriverTest, TestDeadzoneFiltering) {
  // Test dynamic deadzone logic
  // When moving: larger deadzone should filter out small angular inputs
  // When stationary: smaller deadzone should allow precise rotation

  // Test moving scenario (should use larger deadzone)
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.25, 0.3), 0.0);   // filtered out
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.35, 0.3), 0.35);  // passes through

  // Test stationary scenario (should use smaller deadzone)
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.1, 0.15), 0.0);  // filtered out
  EXPECT_EQ(JoyDriverNode::applyDeadzone(0.2, 0.15), 0.2);  // passes through
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

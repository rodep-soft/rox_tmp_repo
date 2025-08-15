#!/usr/bin/env python3
import rclpy
from color_sensor.tca9548a import TCS9548A
from color_sensor.tcs34725 import TCS34725
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import ColorRGBA
from custom_interfaces.action import UpperFunction




class ColorPublisher(Node):

    def __init__(self, tca9548_channel, integration_time=0x00):
        super().__init__("color_publisher")
        self.tca9548 = TCS9548A(1, 0x70)
        self.tca9548_channel = tca9548_channel
        self.tca9548.enable_channel(tca9548_channel)
        self.get_logger().info(f"Enabled TCA9548A channel {tca9548_channel}")
        # Initialize TCS34725 sensor with the specified integration time

        self.TCS34725 = TCS34725(1, 0x29)
        self.TCS34725.change_integration_time(integration_time)
        self.integration_gain_ = (256 - integration_time) * 1024.0
        self.TCS34725.change_gain(0x03)  # Set gain to 60x
        self.TCS34725.enable()
        node_name = f"color_publisher_{tca9548_channel}"
        self.publisher_ = self.create_publisher(ColorRGBA, node_name, 10)


        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Color Publisher Node has been started.")


    def timer_callback(self):
        self.tca9548.enable_channel(self.tca9548_channel)
        msg = ColorRGBA()
        c, r, g, b = self.TCS34725.read_colors()
        msg.r = r / self.integration_gain_  # Normalize to [0, 1]
        msg.g = g / self.integration_gain_  # Normalize to [0, 1]
        msg.b = b / self.integration_gain_  # Normalize to [0, 1]
        msg.a = c / self.integration_gain_  # Normalize to [0, 1]
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Publishing: R:{msg.r} G:{msg.g} B:{msg.b} C:{msg.a}")

    def __del__(self):
        self.TCS34725.disable()


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower")
        self.color_0_subscription = self.create_subscription(
            ColorRGBA, "color_publisher_0", self.color_callback_0, 10
        )
        self.color_1_subscription = self.create_subscription(
            ColorRGBA, "color_publisher_1", self.color_callback_1, 10
        )
        self.color_2_subscription = self.create_subscription(
            ColorRGBA, "color_publisher_2", self.color_callback_2, 10
        )
        self.color_3_subscription = self.create_subscription(
            ColorRGBA, "color_publisher_3", self.color_callback_3, 10
        )
        self.is_enable_subscription = self.create_subscription(
            Bool, "is_linetrace", self.is_enable_callback, 10
        )
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.upper_action_client_ = ActionClient(self, UpperFunction, "upper_function")
        self.outer_diff_publisher_ = self.create_publisher(Float64, "outer_diff", 10)

        self.timer = self.create_timer(0.05, self.publish_twist)
        self.before_diff = None
        self.before_goal_gate = None
        self.integral = 0.0
        self.is_enable = False
        self.is_waiting_for_action = False
        self.is_straight = True
        self.straight_lock_flag = False
        self.get_logger().info("Line Follower Node has been started.")

    def color_callback_0(self, msg):
        self.color_0_ = msg

    def color_callback_1(self, msg):
        self.color_1_ = msg

    def color_callback_2(self, msg):
        self.color_2_ = msg
    
    def color_callback_3(self, msg):
        self.color_3_ = msg
        self.color_3_.a = self.color_3_.a

    def is_enable_callback(self, msg):
        self.is_enable = msg.data

    def publish_twist(self):
        if not self.is_enable:
            #self.get_logger().info("Line trace is disabled, not publishing Twist.")
            self.before_diff = None
            self.integral = 0.0
            self.lock_flag = False
            return
        
        twist = Twist()
        float64 = Float64()
        diff_outer = (self.color_0_.a - self.color_3_.a) / (self.color_0_.a + self.color_3_.a)
        self.color_2_.a = self.color_2_.a + 0.012
        diff = (self.color_1_.a - self.color_2_.a) / (self.color_1_.a + self.color_2_.a)

        if self.before_diff is None:
            self.before_diff = diff
        derivative = diff - self.before_diff
        # if abs(derivative - self.before_diff) <= abs(self.before_diff):
        #     derivative = 0.0

        if abs(diff + self.integral) < abs(self.integral):
            self.integral = self.integral * 0.9

        self.integral += diff

        power = ((6.0 * diff) + (30.0 * derivative) + (0.8 * self.integral))

      

        self.get_logger().info("Publishing Twist: power={}".format(power))
            
        

        x_power = 0.8 - (abs(power) * 0.0)
      
        twist.linear.x = -x_power
        twist.linear.y = power * 1.0 * 0.1
        twist.angular.z = power * 1.0

        float64.data = diff_outer

        if diff_outer < -0.3:
            self.is_waiting_for_action = True
            self.send_upper_function_goal(True)
    
        if self.is_waiting_for_action == True:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0



        #(80.0 * diff_pow) + (1.0 * derivative) + (0.8 * self.integral)
        self.cmd_vel_publisher_.publish(twist)
        self.outer_diff_publisher_.publish(float64)
        self.before_diff = diff

    def send_upper_function_goal(self, is_rising):
        goal = UpperFunction.Goal()
        goal.is_rising = is_rising

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available!')
            return

        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.upper_function_feedback_callback)

        self._send_goal_future.add_done_callback(self.upper_function_goal_response_callback)

    def upper_function_feedback_callback(self, feedback_msg):
        elapsed = feedback_msg.feedback.elapsed_time
        self.get_logger().debug(f"Upper function feedback: Elapsed time {elapsed:.2f} seconds")

    def upper_function_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().debug('Goal rejected')
            return

        self.get_logger().debug('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.upper_function_get_result_callback)

    def upper_function_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().debug(f'Result received: success={result.success}, duration={result.actual_duration:.2f}s')
        self.is_waiting_for_action = False

    
    

def main(args=None):
    rclpy.init(args=args)
    color_publisher_0 = ColorPublisher(0, 0xFC)
    color_publisher_1 = ColorPublisher(1, 0xFC)
    color_publisher_2 = ColorPublisher(2, 0xFC)
    color_publisher_3 = ColorPublisher(3, 0xFC)
    twist_publisher = LineFollower()

    executors = rclpy.executors.SingleThreadedExecutor()
    executors.add_node(color_publisher_0)
    executors.add_node(color_publisher_1)
    executors.add_node(color_publisher_2)
    executors.add_node(color_publisher_3)
    executors.add_node(twist_publisher)
    try:
        executors.spin()
    except KeyboardInterrupt:
        pass
    finally:
        color_publisher_0.destroy_node()
        color_publisher_1.destroy_node()
        color_publisher_2.destroy_node()
        color_publisher_3.destroy_node()
        rclpy.shutdown()
        print("Color Publisher Nodes have been shut down.")


if __name__ == "__main__":
    main()

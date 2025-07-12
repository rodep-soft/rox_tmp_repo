#!/usr/bin/env python3
import rclpy
from color_sensor.tcs34725 import TCS34725
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
from color_sensor.tcs34725 import TCS34725
from color_sensor.tca9548a import TCS9548A

class ColorPublisher(Node):

    def __init__(self, tca9548_channel, integration_time=0x00):
        super().__init__('color_publisher')
        self.tca9548 = TCS9548A(1, 0x70)
        self.tca9548_channel = tca9548_channel
        self.tca9548.enable_channel(tca9548_channel)
        self.get_logger().info(f'Enabled TCA9548A channel {tca9548_channel}')
        # Initialize TCS34725 sensor with the specified integration time

        self.TCS34725 = TCS34725(1, 0x29)
        self.TCS34725.change_integration_time(integration_time)
        self.integration_gain_ = (256 - integration_time) * 1024.0 
        self.TCS34725.change_gain(0x03)  # Set gain to 16x
        self.TCS34725.enable()
        node_name = f'color_publisher_{tca9548_channel}'
        self.publisher_ = self.create_publisher(ColorRGBA, node_name, 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Color Publisher Node has been started.")

    def timer_callback(self):
        self.tca9548.enable_channel(self.tca9548_channel)
        msg = ColorRGBA()
        c, r, g, b = self.TCS34725.read_colors()
        msg.r = r / self.integration_gain_   # Normalize to [0, 1]
        msg.g = g / self.integration_gain_   # Normalize to [0, 1]
        msg.b = b / self.integration_gain_  # Normalize to [0, 1]
        msg.a = c / self.integration_gain_   # Normalize to [0, 1]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: R:{msg.r} G:{msg.g} B:{msg.b} C:{msg.a}")

    def __del__(self):
        self.TCS34725.disable()


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(ColorRGBA, 'color_publisher_0', self.color_callback_0, 10)
        self.subscription = self.create_subscription(ColorRGBA, 'color_publisher_1', self.color_callback_1, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.get_logger().info('Line Follower Node has been started.')

    def color_callback_0(self, msg):
        self.color_0_ = msg
    
    def color_callback_1(self, msg):
        self.color_1_ = msg
    
    def publish_twist(self):
        twist = Twist()
        diff = self.color_0_.a - self.color_1_.a
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = diff * 10.0
        self.publisher_.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    color_publisher_1 = ColorPublisher(0, 0xFC)
    color_publisher_2 = ColorPublisher(1, 0xFC)
    twist_publisher = LineFollower()
    
    executors = rclpy.executors.SingleThreadedExecutor()
    executors.add_node(color_publisher_1)
    executors.add_node(color_publisher_2)
    executors.add_node(twist_publisher)
    try:
        executors.spin()
    except KeyboardInterrupt:
        pass
    finally:
        color_publisher_1.destroy_node()
        color_publisher_2.destroy_node()
        rclpy.shutdown()
        print("Color Publisher Nodes have been shut down.")


if __name__ == "__main__":
    main()

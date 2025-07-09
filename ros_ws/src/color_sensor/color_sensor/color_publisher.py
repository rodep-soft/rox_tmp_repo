#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from color_sensor.tcs34725 import TCS34725

class ColorPublisher(Node):

    def __init__(self, integration_time=0x00):
        super().__init__('color_publisher')
        self.TCS34725 = TCS34725(1, 0x29)
        self.TCS34725.change_integration_time(integration_time)
        self.TCS34725.change_gain(0x02)
        self.TCS34725.enable()
        self.publisher_ = self.create_publisher(ColorRGBA, 'color_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Color Publisher Node has been started.')

    def timer_callback(self):
        msg = ColorRGBA()
        c, r, g, b = self.TCS34725.read_colors()
        msg.r = r / 65535.0  # Normalize to [0, 1]
        msg.g = g / 65535.0  # Normalize to [0, 1]
        msg.b = b / 65535.0  # Normalize to [0, 1]
        msg.a = c / 65535.0  # Normalize to [0, 1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: R:{msg.r} G:{msg.g} B:{msg.b} C:{msg.a}')

    def __del__(self):
        self.TCS34725.disable()
        self.get_logger().info('Color Publisher Node has been shut down.')
    
def main(args=None):
    rclpy.init(args=args)
    color_publisher = ColorPublisher(0xFC)
    rclpy.spin(color_publisher)
    color_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

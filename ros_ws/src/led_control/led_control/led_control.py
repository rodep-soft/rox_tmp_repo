from time import sleep
import rclpy
from gpiozero import Motor
from rclpy.node import Node
from std_msgs.msg import String
import board
import neopixel

class LedControlNode(Node):
    def __init__(self):
        super().__init__("led_control_node")

        self.subscription = self.create_subscription(
            String,
            "/mode",
            self.mode_callback,
            10
        )

        self.pixels = neopixel.NeoPixel(board.D18, 30, brightness=0.5, auto_write=False, pixel_order=neopixel.RGB)

    def mode_callback(self, msg):
        mode = msg.data
        if mode == "STOP":
            self.pixels.fill((255, 0, 0))
            self.pixels.show()
        elif mode == "JOY":
            self.pixels.fill((0, 255, 0))
            self.pixels.show()
        elif mode == "DPAD":
            self.pixels.fill((0, 0, 255))
            self.pixels.show()
        elif mode == "LINETRACE":
            self.pixels.fill((255, 255, 255))
            self.pixels.show()
        else:
            self.pixels.fill((0, 0, 0))
            self.pixels.show()  


def main(args=None):
    rclpy.init(args=args)

    node = LedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


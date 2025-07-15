from time import sleep

import rclpy
from gpiozero import Motor
from rclpy.node import Node
from sensor_msgs.msg import Joy


class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 3)
        self.subscription

    def joy_callback(self, msg):
        motor = Motor(forward=17, backward=27, enable=18)
        motor.forward(speed=msg)


def main(args=None):
    rclpy.init(args=args)

    node = LiftingMotorNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    main()

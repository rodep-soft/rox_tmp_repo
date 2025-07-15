from time import sleep

import rclpy
from gpiozero import Motor
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 1)
        self.motor = Motor(forward=17, backward=27, enable=18)


    def motor_callback(self, msg):
        if msg.drive:
            self.motor.forward(speed=1.0)
        elif msg.stop:
            self.motor.stop()

def main(args=None):
    rclpy.init(args=args)

    node = LiftingMotorNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    main()

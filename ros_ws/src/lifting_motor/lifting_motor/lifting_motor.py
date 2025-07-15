from time import sleep

import rclpy
from gpiozero import Motor
from rclpy.node import Node
from std_msgs.msg import Bool


class LiftingMotorNode(Node):
    def __init__(self):
        super().__init__("lifting_motor_node")
        self.subscription = self.create_subscription(Bool, "/upper_on", self.motor_callback, 1)
        self.motor_on = False
        self.motor = Motor(forward=17, backward=27, enable=18)

    def motor_callback(self, msg):
        # msg is of type std_msgs.msg.Bool
        if msg.data:
            self.motor_on = not self.motor_on
            if self.motor_on:
                self.get_logger().info("Motor ON")
                self.motor.forward(speed=1.0)
            else:
                self.get_logger().info("Motor OFF")
                self.motor.stop()


def main(args=None):
    rclpy.init(args=args)
    node = LiftingMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

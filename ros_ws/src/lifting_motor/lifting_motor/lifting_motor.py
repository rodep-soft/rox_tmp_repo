# from time import sleep
from enum import Enum # Enumを使うために必要

import rclpy
from gpiozero import Motor, OutputDevice, Button
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor


class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 10)
        

        # リミットスイッチの状態管理用のフラグ
        self.is_ejection_maxlim_on = False
        self.is_ejection_minlim_on = False
        self.is_elevation_maxlim_on = False
        self.is_elevation_minlim_on = False

        # モーターの状態管理用のフラグ
        self.is_throwing_motor_on = False
        self.is_ejection_motor_on = False
        self.is_elevation_motor_on = False

        # 前回の状態を保持するための変数
        self.prev_throwing_cmd = False
        self.prev_ejection_cmd = False
        # self.prev_elevation_cmd = False


    # Callback function for UpperMotor messages
    # ここでモーターの制御を行う
    def motor_callback(self, msg):

        self.detect_sw_state() # 全リミットスイッチの状態を検出
        self.detect_motor_state() # 全モーターの状態を検出


        # 押出モーターの実際の動作
        if self.current_state == State.STOPPED:
            if self.is_ejection_motor_on == True:
                self.ejection_motor.stop()
            self.relay_on(msg)
        elif self.current_state == State.TO_MAX:
            self.ejection_motor.forward(speed=1.0)
            # self.detect_edge_and_control_motor(msg)
        elif self.current_state == State.RETURN_TO_MIN:
            self.ejection_motor.backward(speed=1.0)
            self.relay_on(msg)


    # モーターの立ち上がりエッジを検出して制御する
    def detect_edge_and_control_motor(self, msg):
        #　
        if (not self.prev_throwing_cmd) and msg.is_throwing_on:
            self.throwing_motor.on()
            self.prev_throwing_cmd = True

        if (not self.prev_ejection_cmd) and msg.is_ejection_on:
            self.ejection_motor.forward(speed=1.0)
            self.prev_ejection_cmd = True

        

# This code is a ROS2 node that controls a lifting motor using GPIO pins.
# Don't change
def main(args=None):
    rclpy.init(args=args)

    node = LiftingMotorNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

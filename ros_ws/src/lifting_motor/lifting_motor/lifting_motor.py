from time import sleep
from enum import Enum # Enumを使うために必要

import rclpy
from gpiozero import Motor, OutputDevice, Button
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 1)
        
        # -----GPIOピンの設定-----

        # リレーのピン番号
        # ピン番号は仮
        self.relay_pin = 9 # Relay pin number

        # 押出しモーターのピン番号
        # ピン番号は仮である
        self.ejection_motor_forward_pin = 17
        self.ejection_motor_backward_pin = 27
        self.ejection_motor_enable_pin = 18

        # 昇降モーターのピン番号
        # ピン番号は仮
        self.elevation_motor_forward_pin = 22
        self.elevation_motor_backward_pin = 23
        self.elevation_motor_enable_pin = 24

        # リミットスイッチ
        # ピン番号は仮であるため、実際には書き換える必要がある
        self.ejection_maxlim = Button(5)
        self.ejection_minlim = Button(6)
        self.elevation_maxlim = Button(7)
        self.elevation_minlim = Button(8)

        # モーターの初期化
        self.ejection_motor = Motor(forward=self.ejection_motor_forward_pin, backward=self.ejection_motor_backward_pin, enable=self.ejection_motor_enable_pin)
        self.elevation_motor = Motor(forward=self.elevation_motor_forward_pin, backward=self.elevation_motor_backward_pin, enable=self.elevation_motor_enable_pin)
        self.relay_motor = OutputDevice(self.relay_pin, active_high=True, initial_value=False)

        # 押出の状態を定義
        class State(Enum):
            INIT = 0
            STOPPED = 1
            TO_MAX = 2
            RETURN_TO_MIN = 3

        # 初期状態はINIT
        self.current_state = State.INIT

        # リミットスイッチの状態管理用のフラグ
        self.is_ejection_maxlim_on = False
        self.is_ejection_minlim_on = False
        self.is_elevation_maxlim_on = False
        self.is_elevation_minlim_on = False


    # Callback function for UpperMotor messages
    # ここでモーターの制御を行う
    def motor_callback(self, msg):

        self.detect_sw_state() # 全リミットスイッチの状態を検出

        # 射出モーター (リレー駆動)
        if msg.is_throwing_on == 1:
            self.device.on()
            self.get_logger().info("Relay: ON")
        else:
            self.device.off()
            self.get_logger().info("Relay: OFF")

        # 昇降モーター (MD駆動)
        if msg.elevation_mode == 1:
            self.elevation_motor.forward(speed=1.0) # 逆かも
        elif msg.elevation_mode == 0:
            self.elevation_motor.backward(speed=1.0)
        else:
            self.elevation_motor.stop()

        # 状態遷移のロジック
        if self.current_state == self.State.INIT and \
           msg.something_on == True: # これは本当の名前に置き換える
            self.current_state == self.State.STOPPED
        elif self.current_state == self.State.STOPPED and \
           msg.is_throwing_on == True and \
           msg.is_elevation_minlim_on == True and \
           msg.is_ejection_on == True:
            self.current_state = self.State.TO_MAX #状態遷移
        elif self.current_state == self.State.TO_MAX and \
            msg.is_ejection_maxmlim_on == True:
            self.current_state = self.State.RETURN_TO_MIN #状態遷移
            # Action:
            self.relay_motor.off() # 押し出しが完了したらリレーをオフにする     
        elif self.current_state == self.State.RETURN_TO_MIN and msg.is_ejection_minlim_on == True:
            self.current_state = self.State.STOPPED #状態遷移

        # 押出モーターの実際の動作
        if self.current_state == self.State.STOPPED:
            self.ejection_motor.stop()
            self.relay_on()
        elif self.current_state == self.State.TO_MAX:
            self.ejection_motor.forward(speed=1.0)
        elif self.current_state == self.State.RETURN_TO_MIN:
            self.ejection_motor.backward(speed=1.0)
            self.relay_on()


    def relay_on(self):
        # リレーはSTOPPEDまたはRETURN_TO_MIN状態のときとか駆動することはできない
        if self.is_throwing_on == True:
            self.relay_motor.on()
        # else:
        #     self.relay_motor.off()

    # リミットスイッチの状態を検出するメソッド
    def detect_sw_state(self):
        if self.ejection_maxlim.is_pressed:
            self.get_logger().info("Ejection Max Limit Switch is pressed")
            self.is_ejection_maxlim_on = True
        
        if self.ejection_minlim.is_pressed:
            self.get_logger().info("Ejection Min Limit Switch is pressed")
            self.is_ejection_minlim_on = True

        if self.elevation_maxlim.is_pressed:
            self.get_logger().info("Elevation Max Limit Switch is pressed")
            self.is_elevation_maxlim_on = True
        
        if self.elevation_minlim.is_pressed:
            self.get_logger().info("Elevation Min Limit Switch is pressed")
            self.is_elevation_minlim_on = True



# This code is a ROS2 node that controls a lifting motor using GPIO pins.
def main(args=None):
    rclpy.init(args=args)

    node = LiftingMotorNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    main()

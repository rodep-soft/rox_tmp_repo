# from time import sleep
from enum import Enum # Enumを使うために必要

import rclpy
from gpiozero import Motor, OutputDevice, Button
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor

# 押出の状態を定義
class State(Enum):
    INIT = 0
    STOPPED = 1
    TO_MAX = 2
    RETURN_TO_MIN = 3
class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 10)
        
        # -----GPIOピンの設定-----

        # リレーのピン番号
        # ピン番号は仮
        self.relay_pin = 9 # 射出用モーターを駆動するためのリレーのGPIOピン番号

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
        self.throwing_motor = OutputDevice(self.relay_pin, active_high=True, initial_value=False)

        # 初期状態はINIT
        self.current_state = State.INIT

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

        # # 射出モーター (リレー駆動)
        # if msg.is_throwing_on == 1:
        #     self.device.on()
        #     self.get_logger().info("Relay: ON")
        # else:
        #     self.device.off()
        #     self.get_logger().info("Relay: OFF")

        # # 昇降モーター (MD駆動)
        # if msg.elevation_mode == 1:
        #     self.elevation_motor.forward(speed=1.0) # 逆かも
        # elif msg.elevation_mode == 0:
        #     self.elevation_motor.backward(speed=1.0)
        # else:
        #     self.elevation_motor.stop()


        # 状態遷移のロジック
        if self.current_state == State.INIT and \
           msg.something_on == True: # これは本当の名前に置き換える
            self.current_state = State.STOPPED
        elif self.current_state == State.STOPPED and \
           self.is_throwing_motor_on == True and \
           self.is_elevation_minlim_on == True and \
           msg.is_ejection_on == True:
            # 射出モーターがONで、昇降が完全に下がっていて、押し出しがONになったとき遷移
            self.current_state = State.TO_MAX #状態遷移
        elif self.current_state == State.TO_MAX and \
            self.is_ejection_maxlim_on == True:
            # 押し出しが最大まで伸び切ってリミットスイッチを押したとき、遷移
            self.current_state = State.RETURN_TO_MIN #状態遷移
            # Action:
            self.throwing_motor.off() # 押し出しが完了したらリレーをオフにする     
        elif self.current_state == State.RETURN_TO_MIN and \
            self.is_ejection_minlim_on == True:
            # 押し出しが完全に引っ込んで最小リミットスイッチを押したら遷移
            self.current_state = State.STOPPED #状態遷移

        # 押出モーターの実際の動作
        if self.current_state == State.STOPPED:
            if self.is_ejection_motor_on == True:
                self.ejection_motor.stop()
            self.relay_on(msg)
        elif self.current_state == State.TO_MAX:
            self.ejection_motor.forward(speed=1.0)
        elif self.current_state == State.RETURN_TO_MIN:
            self.ejection_motor.backward(speed=1.0)
            self.relay_on(msg)


    def relay_on(self, msg):
        # リレーはSTOPPEDまたはRETURN_TO_MIN状態のときとか駆動することはできない
        if msg.is_throwing_on == True and self.is_throwing_motor_on == False:
            self.throwing_motor.on()
        # else:
        #     self.relay_motor.off()


    # リミットスイッチの状態を検出するメソッド
    def detect_sw_state(self):
        self.is_ejection_maxlim_on = self.ejection_maxlim.is_pressed
        self.is_ejection_minlim_on = self.ejection_minlim.is_pressed
        self.is_elevation_maxlim_on = self.elevation_maxlim.is_pressed
        self.is_elevation_minlim_on = self.elevation_minlim.is_pressed


    # モーターの状態を検出するメソッド
    # モーターが動作しているかどうかを確認する　
    def detect_motor_state(self):
        self.is_throwing_motor_on = self.throwing_motor.is_active
        self.is_ejection_motor_on = self.ejection_motor.is_active
        self.is_elevation_motor_on = self.elevation_motor.is_active

    def detect_edge_and_control_motor(self, msg):
        if (not self.prev_throwing_cmd) and msg.is_throwing_on:
            self.throwing_motor.on()
            self.prev_throwing_cmd = True

        if (not self.prev_ejection_cmd) and msg.is_ejection_on:
            self.ejection_motor.forward(speed=1.0)
            self.prev_ejection_cmd = True
        



# This code is a ROS2 node that controls a lifting motor using GPIO pins.
def main(args=None):
    rclpy.init(args=args)

    node = LiftingMotorNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

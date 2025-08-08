from time import sleep
from enum import Enum

import rclpy
from gpiozero import Motor, OutputDevice
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 1)
        self.relay_pin = 9 # Relay pin number
        self.elevation_motor = Motor(forward=17, backward=27, enable=18)
        self.ejection_motor = Motor(forward=22, backward=23, enable=24) # これは仮のピン番号
        self.relay_motor = OutputDevice(self.relay_pin, active_high=True, initial_value=False)

        # 押出の状態を定義
        class State(Enum):
            INIT = 0
            STOPPED = 1
            TO_MAX = 2
            RETURN_TO_MIN = 3

        # 初期状態はINIT
        self.current_state = State.INIT



    def motor_callback(self, msg):
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

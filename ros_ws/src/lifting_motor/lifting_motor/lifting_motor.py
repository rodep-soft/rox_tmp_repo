
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor
from state_machine import State, StateMachine
from motor_driver import MotorDriver

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 10)
        
        # State and MotorDriver initialization
        self.state_machine = StateMachine()
        self.motor_driver = MotorDriver()

    # Callback function for UpperMotor messages
    # ここでモーターの制御を行う
    def motor_callback(self, msg):
        # スイッチの状態を取得
        sw = self.motor_driver.get_switch_states()

        # 状態遷移の更新用
        inputs = {
            "start": msg.something_on, # INIT<-->STOPPED
            "throwing_on": msg.is_throwing_on,
            "elev_min": sw["elev_min"],
            "eject_on": msg.is_ejection_on,
            "eject_max": sw["eject_max"],
            "eject_min": sw["eject_min"],
        }

        new_state = self.state_machine.update_state(inputs)
        self.get_logger().info(f"Current State: {self.state_machine.get_state_name()}")


        # 状態遷移後の実際のモーター制御
        if new_state == State.TO_MAX:
            self.motor_driver.ejection_forward()
        elif new_state == State.RETURN_TO_MIN:
            self.motor_driver.ejection_backward()


    # モーターの立ち上がりエッジを検出して制御する
    def detect_edge_and_control_motor(self, msg):
        #　
        if (not self.prev_throwing_cmd) and msg.is_throwing_on:
            self.throwing_motor.on()
            self.prev_throwing_cmd = True

        if (not self.prev_ejection_cmd) and msg.is_ejection_on:
            self.ejection_motor.forward(speed=1.0)
            self.prev_ejection_cmd = True

        
# ========================
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


import rclpy
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor
from state_machine import State, StateMachine
from motor_driver import MotorDriver

# lifting_motorノードのメインのプログラム
# state_machine.pyに状態とその遷移のロジックが,
# motor_driver.pyにモーターの制御ロジックが含まれる

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 10)
        
        # State and MotorDriver initialization
        self.state_machine = StateMachine()
        self.motor_driver = MotorDriver()
        
        # エッジ検出用の前回値保持
        self.prev_throwing_on = False
        self.prev_ejection_on = False
        self.elevation_warning_logged = False  # 昇降警告ログのフラグ
        
        # ハードウェア検証
        # リミットスイッチとモーターの状態の読み取りが可能か確認する
        if not self.motor_driver.validate_hardware():
            self.get_logger().error("ハードウェアの初期化に失敗しました")
        else:
            self.get_logger().info("ハードウェアの初期化が完了しました")

    # Callback function for UpperMotor messages
    # ここでモーターの制御を行う
    def motor_callback(self, msg):
        try:
            # スイッチの状態を取得
            sw = self.motor_driver.get_switch_states()

            # エッジ検出
            throwing_edge = msg.is_throwing_on and not self.prev_throwing_on
            ejection_edge = msg.is_ejection_on and not self.prev_ejection_on
            
            # 前回値を更新
            self.prev_throwing_on = msg.is_throwing_on
            self.prev_ejection_on = msg.is_ejection_on

            # 状態遷移の更新用
            inputs = {
                "is_system_ready": msg.is_system_ready,
                "is_throwing_motor_on": self.motor_driver.get_motor_states()["is_throwing_motor_running"],
                "is_elevation_minlim_on": sw["elevation_min"],
                "is_ejection_on": ejection_edge,  # エッジ検出結果を使用
                "is_ejection_maxlim_on": sw["ejection_max"],
                "is_ejection_minlim_on": sw["ejection_min"],
                "safety_reset_button": False,  # TODO: 実際のボタン状態に置き換え
                "emergency_stop": False,       # TODO: 実際の緊急停止状態に置き換え
            }

            # 状態遷移の更新
            self.state_machine.update_state(inputs)
            current_state = self.state_machine.get_current_state()
            
            # 状態が変化したときのみログ出力（ログの削減）
            if self.state_machine.has_state_changed():
                self.get_logger().info(f"State Changed: {self.state_machine.get_previous_state().name} -> {self.state_machine.get_state_name()}")

            # TO_MAXに遷移したときの一度だけの処理（副作用）
            if self.state_machine.just_entered_state(State.TO_MAX):
                self.motor_driver.throwing_off()  # リレー停止（一度だけ）
                self.get_logger().info("TO_MAX遷移: リレーを停止しました")

            # 射出用リレーのモーメンタリ制御（STOPPEDまたはRETURN_TO_MIN状態でのみ）
            if current_state in [State.STOPPED, State.RETURN_TO_MIN] and throwing_edge:
                self.motor_driver.throwing_on()
                self.get_logger().info("射出リレーをONにしました")

            # 状態に応じた押出モーター制御
            if current_state == State.STOPPED:
                self.motor_driver.ejection_stop()
            elif current_state == State.TO_MAX:
                self.motor_driver.ejection_forward()  # 射出駆動
            elif current_state == State.RETURN_TO_MIN:
                self.motor_driver.ejection_backward()

            # 昇降モーター制御（motor_driverに委譲）
            force_descending = self.motor_driver.elevation_control(msg.elevation_mode, current_state)
            if force_descending and not self.elevation_warning_logged:
                self.get_logger().warning(f"非INIT状態({current_state.name})のため昇降を強制的に下降位置へ移動中")
                self.elevation_warning_logged = True
            elif not force_descending and current_state == State.INIT:
                self.elevation_warning_logged = False  # INIT状態になったら警告フラグをリセット

                
        except Exception as e:
            self.get_logger().error(f"Motor callback error: {e}")
            # エラー時は安全のため全モーター停止
            try:
                self.motor_driver.stop_all_motors()
            except:
                pass


    # # モーターの立ち上がりエッジを検出して制御する
    # def detect_edge_and_control_motor(self, msg):
    #     #　
    #     if (not self.prev_throwing_cmd) and msg.is_throwing_on:
    #         self.throwing_motor.on()
    #         self.prev_throwing_cmd = True

    #     if (not self.prev_ejection_cmd) and msg.is_ejection_on:
    #         self.ejection_motor.forward(speed=1.0)
    #         self.prev_ejection_cmd = True

        


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
# ========================
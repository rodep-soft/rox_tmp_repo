
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import UpperMotor
# from state_machine import State, StateMachine
# from motor_driver import MotorDriver
from lifting_motor.state_machine import State, StateMachine
from lifting_motor.motor_driver import MotorDriver
from time import sleep

from custom_interfaces.action import UpperFunction
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse

from std_msgs.msg import String

# lifting_motorノードのメインのプログラム
# state_machine.pyに状態とその遷移のロジックが,
# motor_driver.pyにモーターの制御ロジックが含まれる

class LiftingMotorNode(Node):

    def __init__(self):
        super().__init__("lifting_motor_node")
        # UpperMotor msgのsubscription
        self.subscription = self.create_subscription(UpperMotor, "/upper_motor", self.motor_callback, 10)

        self.publisher_ = self.create_publisher(String, "/lifting_mode", 10) # led pattern
        
        # State and MotorDriver initialization
        self.state_machine = StateMachine()
        self.motor_driver = MotorDriver()
        
        # エッジ検出用の前回値保持
        self.prev_throwing_on = False
        self.prev_ejection_on = False

        self.prev_init_on = False # 初期化状態の前回値保持

        self.elevation_warning_logged = False  # 昇降警告ログのフラグ
        self.ejection_blocking_logged = False  # 押し出しブロック警告ログのフラグ
        
        # ハードウェア検証
        # リミットスイッチとモーターの状態の読み取りが可能か確認する
        if not self.motor_driver.validate_hardware():
            self.get_logger().error("ハードウェアの初期化に失敗しました")
        else:
            self.get_logger().info("ハードウェアの初期化が完了しました")

        self._action_server = ActionServer(
            self,
            UpperFunction,
            'upper_function',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):

        if goal_request.is_rising:
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback(self, goal_handle):
        """昇降アクションの実行"""
        self.get_logger().info("昇降アクション開始")
        
        goal = goal_handle.request
        result = UpperFunction.Result()
        feedback = UpperFunction.Feedback()
        
        start_time = self.get_clock().now()
        timeout_duration = 10.0  # 10秒タイムアウト
        last_feedback_time = 0.0  # 最後にフィードバックを送信した時刻
        
        # 昇降制御開始（motor_driver.pyの仕様に合わせる）
        # is_rising=True → elevation_mode=1 (上昇)
        # is_rising=False → elevation_mode=0 (下降)
        elevation_mode = 1 if goal.is_rising else 0
        
        while rclpy.ok():
            # キャンセル確認
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.actual_duration = 0.0
                self.get_logger().info("昇降アクションがキャンセルされました")
                return result
            
            # タイムアウト確認
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_duration:
                goal_handle.abort()
                result.success = False
                result.actual_duration = elapsed
                self.get_logger().error("昇降アクションがタイムアウトしました")
                return result
            
            # 昇降制御実行
            current_state = self.state_machine.get_current_state()
            elevation_status = self.motor_driver.elevation_control(elevation_mode, current_state)
            
            # フィードバック送信（1秒に1回）
            if elapsed - last_feedback_time >= 1.0:
                feedback.elapsed_time = elapsed
                goal_handle.publish_feedback(feedback)
                last_feedback_time = elapsed
            
            # 完了確認
            if elevation_status == "stopped":
                result.success = True
                result.actual_duration = elapsed
                goal_handle.succeed()
                self.get_logger().info(f"昇降アクション完了: {elapsed:.2f}秒")
                return result
            
            # 短い待機
            sleep(0.1)
        
        # 異常終了
        result.success = False
        result.actual_duration = elapsed
        goal_handle.abort()
        return result

    def cancel_callback(self, goal_handle):
        """アクションキャンセル処理"""
        self.get_logger().info("昇降アクションのキャンセル要求を受諾")
        return CancelResponse.ACCEPT

    
    # def cancel_callback(self, goal_handle):
    #     return CancelResponse.ACCEPT


    # Callback function for UpperMotor messages
    # ここでモーターの制御を行う
    def motor_callback(self, msg):

        try:
            # スイッチの状態を取得
            sw = self.motor_driver.get_switch_states()

            # エッジ検出
            throwing_edge = msg.is_throwing_on and not self.prev_throwing_on
            ejection_edge = msg.is_ejection_on and not self.prev_ejection_on

            init_edge = msg.is_system_ready and not self.prev_init_on
            
            # 前回値を更新
            self.prev_throwing_on = msg.is_throwing_on
            self.prev_ejection_on = msg.is_ejection_on

            self.prev_init_on = msg.is_system_ready

            # 状態遷移の更新用
            inputs = {
                # "is_system_ready": msg.is_system_ready,
                "is_system_ready": init_edge,
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

            # TO_MAXに遷移したときの一度だけの処理(副作用)
            # STOPPEDでボタンが押されると、ここでリレーON + 2秒待機 + 押し出し動作開始
            if self.state_machine.just_entered_state(State.TO_MAX):
                self.motor_driver.throwing_on()
                self.get_logger().info("TO_MAX遷移: リレーをONにしました")
                sleep(1)  # 2秒待機後、下記の押し出し制御で前進開始

            # RETURN_TO_MINに遷移したときの一度だけの処理（副作用）
            if self.state_machine.just_entered_state(State.RETURN_TO_MIN):
                self.motor_driver.throwing_off()  # リレー停止（一度だけ）
                self.get_logger().info("RETURN_TO_MIN遷移: リレーを停止しました")

            # # 射出用リレーの制御（状態とエッジ検出に基づく）
            # if throwing_edge:
            #     if current_state in [State.STOPPED, State.RETURN_TO_MIN]:
            #         # 射出可能な状態でボタンが押された場合
            #         if not self.motor_driver.get_motor_states()["is_throwing_motor_running"]:
            #             # 現在停止中なら開始
            #             self.motor_driver.throwing_on()
            #             self.get_logger().info("射出リレーをONにしました")
            #         else:
            #             # 現在動作中なら停止
            #             self.motor_driver.throwing_off()
            #             self.get_logger().info("射出リレーをOFFにしました")

            # 状態に応じた押出モーター制御
            if current_state == State.STOPPED:
                # STOPPEDステート: ejection_minまで後退させる
                # ボタン押下時はstate_machineがTO_MAXに遷移させる
                if not self.motor_driver.get_switch_states()["ejection_min"]:
                    self.motor_driver.ejection_backward()
                else:
                    self.motor_driver.ejection_stop()
            elif current_state == State.TO_MAX:
                self.motor_driver.ejection_forward()  # 射出駆動
            elif current_state == State.RETURN_TO_MIN:
                self.motor_driver.ejection_backward()

            # 昇降モーター制御（motor_driverに委譲）
            elevation_status = self.motor_driver.elevation_control(msg.elevation_mode, current_state)
            
            # 昇降制御のログ管理
            if elevation_status == "force_descending" and not self.elevation_warning_logged:
                self.get_logger().warning(f"非INIT状態({current_state.name})のため昇降を強制的に下降位置へ移動中")
                self.elevation_warning_logged = True
            elif elevation_status == "blocked_by_ejection" and not self.ejection_blocking_logged:
                self.get_logger().warning("押し出しが引っ込んでいないため昇降できません")
                self.ejection_blocking_logged = True
            elif current_state == State.INIT and elevation_status in ["elevating", "descending", "stopped"]:
                # INIT状態で正常動作時は警告フラグをリセット
                self.elevation_warning_logged = False
                self.ejection_blocking_logged = False

                
        except Exception as e:
            self.get_logger().error(f"Motor callback error: {e}")
            # エラー時は安全のため全モーター停止
            try:
                self.motor_driver.stop_all_motors()
            except:
                pass

        mode_msg = String()
        mode_msg.data = self.state_machine.get_state_name()
        self.publisher_.publish(mode_msg)

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
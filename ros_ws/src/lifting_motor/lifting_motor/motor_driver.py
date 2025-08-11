from gpiozero import Motor, OutputDevice, Button

# このコードには状態遷移に関する情報、またメインのロジックは絶対に含まないこと
# relay, MD, Buttonの制御、要するにGPIO周りの制御のみを担当するクラスを作成する

class MotorDriver:
    """モーターの制御とボタンの状態検知のみを行うクラス"""
    def __init__(self):
        # -----GPIO configuration-----
        # リレーの制御用ピン
        self.relay_pin = 19

        # 押出しモーター制御用ピン
        self.ejection_motor_forward_pin = 5
        self.ejection_motor_backward_pin = 6
        self.ejection_motor_enable_pin = 13

        # 昇降モーター制御用ピン
        self.elevation_motor_forward_pin = 17
        self.elevation_motor_backward_pin = 16
        self.elevation_motor_enable_pin = 12

        # -----GPIO configuration END-----

        # -----Motor and Switch initialization-----
        # Initialize relay
        self.throwing_motor = OutputDevice(self.relay_pin)

        # Initialize motors
        self.ejection_motor = Motor(forward=self.ejection_motor_forward_pin, backward=self.ejection_motor_backward_pin, enable=self.ejection_motor_enable_pin)
        self.elevation_motor = Motor(forward=self.elevation_motor_forward_pin, backward=self.elevation_motor_backward_pin, enable=self.elevation_motor_enable_pin)

        # Limit switches
        self.ejection_maxlim = Button(26)
        self.ejection_minlim = Button(27)
        self.elevation_maxlim = Button(25)
        self.elevation_minlim = Button(24)

        # -----Motor and Switch initialization END-----


    def get_switch_states(self) -> dict:
        return {
            "ejection_max": self.ejection_maxlim.is_pressed,
            "ejection_min": self.ejection_minlim.is_pressed,
            "elevation_max": self.elevation_maxlim.is_pressed,
            "elevation_min": self.elevation_minlim.is_pressed
        }
    
    # 射出はリレー制御
    # 昇降と押出しはMDで制御
    # 昇降が動いているかの確認は恐らく使わないと思う
    def get_motor_states(self) -> dict:
        return {
            "is_ejection_motor_running": self.ejection_motor.is_active,
            "is_elevation_motor_running": self.elevation_motor.is_active,
            "is_throwing_motor_running": self.throwing_motor.is_active
        }
    
    # API functions for motor controller

    def throwing_on(self):
        self.throwing_motor.on()

    def throwing_off(self):
        self.throwing_motor.off()

    def ejection_forward(self, speed=1.0):
        self.ejection_motor.forward(speed=speed)

    def ejection_backward(self, speed=1.0):
        self.ejection_motor.backward(speed=speed)

    def ejection_stop(self):
        self.ejection_motor.stop()

    def elevation_forward(self, speed=1.0):
        self.elevation_motor.forward(speed=speed)

    def elevation_backward(self, speed=0.5):
        self.elevation_motor.backward(speed=speed)

    def elevation_stop(self):
        self.elevation_motor.stop()

    def elevation_control(self, elevation_mode, current_state):
        """昇降モーター制御（INIT状態でのみ駆動可能、それ以外は強制的に下降位置へ）"""
        switch_states = self.get_switch_states()
        
        if current_state.name == "INIT":  # Enumの比較
            # INIT状態でのみ自由に昇降可能
            if elevation_mode == 1 and not switch_states["elevation_max"] and switch_states["ejection_min"]:
                # 上昇（最大リミットに達していない かつ 押し出しが引っ込んでいる場合のみ）
                self.elevation_forward()
                return "elevating"
            elif elevation_mode == 0 and not switch_states["elevation_min"]:
                # 下降（最小リミットに達していない場合のみ）
                self.elevation_backward()
                return "descending"
            elif elevation_mode == 1 and not switch_states["ejection_min"]:
                # 上昇要求があるが押し出しが引っ込んでいない場合は停止
                self.elevation_stop()
                return "blocked_by_ejection"
            else:
                # 停止（mode=2 または ボタンが押されていない時 または リミットスイッチ押下時）
                self.elevation_stop()
                return "stopped"
        else:
            # INIT以外の状態では強制的に下降位置へ
            if not switch_states["elevation_min"]:
                # 最小位置に達していない場合、強制的に下降
                self.elevation_backward()
                return "force_descending"  # 強制下降中であることを返す
            else:
                # 最小位置に達している場合は停止
                self.elevation_stop()
                return "at_bottom"  # 下降完了

    def stop_all_motors(self):
        """緊急時用：全モーター停止"""
        self.ejection_motor.stop()
        self.elevation_motor.stop()
        self.throwing_motor.off()

    def get_all_states(self) -> dict:
        """全状態を一度に取得（デバッグ用）"""
        return {
            **self.get_switch_states(),
            **self.get_motor_states()
        }
    
    def validate_hardware(self) -> bool:
        """ハードウェアの基本チェック"""
        try:
            # 基本的な読み取りテスト
            switches = self.get_switch_states()
            motors = self.get_motor_states()
            return True
        except Exception:
            return False

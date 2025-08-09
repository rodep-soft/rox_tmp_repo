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
        self.elevation_motor_forward_pin = 16
        self.elevation_motor_backward_pin = 27
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





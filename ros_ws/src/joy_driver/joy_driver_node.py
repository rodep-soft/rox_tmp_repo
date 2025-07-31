import gpiozero as gpio
from time import sleep
from gpiozero import Button

class MotorUpDown:
    
    def __init__(self):
        self.motor_A = gpio.DigitalOutputDevice(MotorUpDown.__MOTOR_A_SPIN_POSITIVE)
        self.motor_B = gpio.DigitalOutputDevice(MotorUpDown.__MOTOR_B_SPIN_NEGATIVE)
        self.motor_PWM = gpio.PWMOutputDevice(MotorUpDown.__MOTOR_PWM_PIN)
        self.motor2_A = gpio.DigitalOutputDevice(MotorUpDown.__MOTOR2_A_SPIN_POSITIVE)
        self.motor2_B = gpio.DigitalOutputDevice(MotorUpDown.__MOTOR2_B_SPIN_NEGATIVE)
        self.motor2_PWM = gpio.PWMOutputDevice(MotorUpDown.__MOTOR2_PWM_PIN)
        self.relay = gpio.DigitalOutputDevice(MotorUpDown.__relay_pin)

    #Setting motor pin of motor1
    __MOTOR_A_SPIN_POSITIVE = 23
    __MOTOR_B_SPIN_NEGATIVE = 24
    __MOTOR_PWM_PIN = 25
    #Setting motor pin of motor2
    __MOTOR2_A_SPIN_POSITIVE = 16
    __MOTOR2_B_SPIN_NEGATIVE = 20
    __MOTOR2_PWM_PIN = 21

    #リミットスイッチのボタン
    __button2 = Button(2)

    #何かのスイッチ
    __button3 = Button(3)
    __button4 = Button(4)
    __button5 = Button(5)

    __relay_pin = 17

    __HIGH = 1
    __LOW = 0

    def lifting_mechanism_up(self):
        #リミットスイッチがonかどうかの確認
        if MotorUpDown.__button2.is_pressed:
            return
        else:
            #リミットスイッチがoffだったら昇降機構のモーターが上がる
            self.motor_A.on()
            self.motor_B.off()
            self.motor2_A.on()
            self.motor2_B.off()

    #どこまで下げるのかな？
    def lifting_mechanism_down(self):
        self.motor_A.off()
        self.motor_B.on()
        self.motor2_A.off()
        self.motor2_B.on()

    #何かの時に使うかもだけどどういうときに使うかはまだ思いついていない
    def lifting_mechanism_stop(self):
        self.motor_A.off()
        self.motor_B.off()
        self.motor2_A.off()
        self.motor2_B.off()
    
    def set_motor_speed(self, speed1, speed2):
        """モーターの速度を設定 (0.0-1.0)"""
        # 速度を0.0-1.0の範囲に制限
        speed1 = max(0.0, min(1.0, speed1))
        speed2 = max(0.0, min(1.0, speed2))
        
        self.motor_PWM.value = speed1
        self.motor2_PWM.value = speed2

    #GPIOリソースの解放
    def cleanup(self):
        self.motor_A.close()
        self.motor_B.close()
        self.motor_PWM.close()
        self.motor2_A.close()
        self.motor2_B.close()
        self.motor2_PWM.close()
        self.relay.close()


# 実際にモーターを動かすテストコード
def test_motor():
    """モーターのテスト関数"""
    print("モーターテストを開始します...")
    
    motor = MotorUpDown()
    
    try:
        # 速度を50%に設定
        motor.set_motor_speed(0.5, 0.5)
        
        # 上昇テスト
        print("上昇テスト開始")
        motor.lifting_mechanism_up()
        sleep(2)
        
        # 停止
        print("停止")
        motor.lifting_mechanism_stop()
        sleep(1)
        
        # 下降テスト
        print("下降テスト開始")
        motor.lifting_mechanism_down()
        sleep(2)
        
        # 停止
        print("停止")
        motor.lifting_mechanism_stop()
        
    except KeyboardInterrupt:
        print("テストが中断されました")
    
    finally:
        # リソースを解放
        motor.cleanup()
        print("モーターテスト終了")

# ジョイスティック制御の例（後で実装予定）
def joystick_control_example():
    """将来的なジョイスティック制御の例"""
    # 仮のジョイスティックデータ
    L1 = 4  # L1トリガーの軸番号
    L2 = 5  # L2トリガーの軸番号
    
    # 仮の軸データ（実際はジョイスティックから取得）
    axis = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0]
    
    motor = MotorUpDown()
    
    # L1トリガーで上昇
    if axis[L1] < 0.5:  # トリガーが押された時
        motor.lifting_mechanism_up()
    # L2トリガーで下降
    elif axis[L2] < 0.5:  # トリガーが押された時
        motor.lifting_mechanism_down()
    else:
        motor.lifting_mechanism_stop()

# メイン実行部分
if __name__ == "__main__":
    # テスト実行
    test_motor()
    
    # 以下は後で実装予定のコード（コメントアウト）
    # joystick_control_example()


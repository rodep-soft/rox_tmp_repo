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

    def lifting_mechanum_up(self):
        #リミットスイッチがonかどうかの確認
        #これでいいのかは知らん
        if  MotorUpDown.__button2 == MotorUpDown.__HIGH:
            return
        else:
            #リミットスイッチがoffだったら昇降機構のモーターが上がる
            self.motor_A.on()
            self.motor_B.off()
            self.motor2_A.on()
            self.motor2_B.off()
            #リミットスイッチがonになったら昇降モーターが止まる
            if self.__button2 == MotorUpDown.__HIGH:
                self.motor_A.on()
                self.motor_B.on()
                self.motor2_A.on()
                self.motor2_B.on()

        if(buttons[sankaku] == MotorUpDown.__HIGH):
            

    #どこまで下げるのかな？
    def lifting_mechanum_down(self):
        self.motor_A.off()
        self.motor_B.on()
        self.motor2_A.off()
        self.motor2_B.on()

    #何かの時に使うかもだけどどういうときに使うかはまだ思いついていない
    def lifting_mechanum_stop(self):
        self.motor_A.off()
        self.motor_B.off()
        self.motor2_A.off()
        self.motor2_B.off()

    #GPIOリソースの解放
    def cleanup(self):
        self.motor_A.close()
        self.motor_B.close()
        self.motor_PWM.close()
        self.motor2_A.close()
        self.motor2_B.close()
        self.motor2_PWM.close()
        self.relay.close()


# テスト用のコード（後でaxis[L1]を実装予定）
def testcode():
    Motor = MotorUpDown()

    #loopする関数の中で書く予定なのかもしれないの中で書きたいな
    if axis[L1] == 1:
        Motor.motor1_up()


    if axis[L2] == 1:
        Motor.motor_down()

    
    
    
    
    
    
    #finish
    Motor.cleanup()

    


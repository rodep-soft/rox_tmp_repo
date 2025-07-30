import gpiozero as gpio
from time import sleep

motor_A_PIN = 17  # モーターAの方向入力A
motor_B_PIN = 27  # モーターAの方向入力B
motor_PWM_PIN = 22  # モーターAのPWM入力

motor2_A_PIN = 23  # モーターBの方向入力A
motor2_B_PIN = 24  # モーターBの方向入力B
motor2_PWM_PIN = 25  # モーターBのPWM入力

motor_A = gpio.DigitalOutputDevice(motor_A_PIN)
motor_B = gpio.DigitalOutputDevice(motor_B_PIN)
motor_PWM = gpio.PWMOutputDevice(motor_PWM_PIN)

motor2_A = gpio.DigitalOutputDevice(motor2_A_PIN)
motor2_B = gpio.DigitalOutputDevice(motor2_B_PIN)
motor2_PWM = gpio.PWMOutputDevice(motor2_PWM_PIN)

try:
    while True:
        motor_A.on()
        motor_B.off()
        motor_PWM.value = 0.1  # フルスピード
        motor2_A.on()
        motor2_B.off()
        motor2_PWM.value = 0.1  # フルスピード
        print("モーターA: フォワード")
        sleep(2)
        motor_A.off()
        motor_B.on()
        motor_PWM.value = 1  # フルスピード
        motor2_A.off()
        motor2_B.on()
        motor2_PWM.value = 1  # フルスピード
        print("モーターA: リバース")
        sleep(2)

finally:
    motor_A.close()
    motor_B.close()
    motor_PWM.close()
    motor2_A.close()
    motor2_B.close()
    motor2_PWM.close()
    print("モーターテスト終了")
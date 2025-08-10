from gpiozero import Motor
from time import sleep


elevation_motor = Motor(forward=16, backward=17, enable=12)


sleep(1)
elevation_motor.forward(speed=0.1)
sleep(3)
elevation_motor.backward(speed=0.1)
sleep(3)
elevation_motor.stop()



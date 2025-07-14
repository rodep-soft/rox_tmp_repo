from gpiozero import Motor
from time import sleep

# Use pins 17 for forward, 27 for backward, and 18 for PWM enable
motor = Motor(forward=17, backward=27, enable=18)

print("Running motor forward at 75% speed...")
motor.forward(speed=0.75)
sleep(5)

print("Reversing direction to backward at 50% speed...")
motor.backward(speed=0.5)
sleep(5)

print("Stopping motor...")
motor.stop()

# You can also use the value property for continuous control
print("\nSweeping speed from full backward to full forward...")
for speed in range(-100, 101):
    motor.value = speed / 100.0
    sleep(0.05)

print("\nCleaning up.")
motor.close()

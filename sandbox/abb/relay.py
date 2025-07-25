from gpiozero import OutputDevice
from time import sleep  

RELAY_PIN = 9

device = OutputDevice(RELAY_PIN, active_high=True, initial_value=False)

while True:
    device.off()
    print("Device : Off")
    sleep(2)

    device.on()
    print("Device : on")
    sleep(2)



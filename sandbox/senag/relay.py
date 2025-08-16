from time import sleep

import gpiozero as gpio

relay_pin = 17  # GPIOピン番号を指定
relay = gpio.DigitalOutputDevice(relay_pin)
try:
    while True:
        relay.on()  # リレーをオンにする
        print("リレーオン")
        sleep(2)  # 2秒待機
        relay.off()  # リレーをオフにする
        print("リレーオフ")
        sleep(2)  # 2秒待機
finally:
    relay.close()

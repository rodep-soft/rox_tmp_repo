# import necessary libraries
#
# pip install adafruit-circuitpython-neopixel


import time

import board
import neopixel

pixels = neopixel.NeoPixel(
    board.D23, 6, brightness=0.5, auto_write=False, pixel_order=neopixel.RGB
)

while True:
    pixels.fill((255, 0, 0))  # 赤
    pixels.show()
    time.sleep(1)
    pixels.fill((0, 255, 0))  # 緑
    pixels.show()
    time.sleep(1)
    pixels.fill((0, 0, 255))  # 青
    pixels.show()
    time.sleep(1)
    print("LED test completed.")

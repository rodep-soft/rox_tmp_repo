import board
import neopixel
# import time

pixels = neopixel.NeoPixel(board.D23, 6, brightness=0.5, auto_write=False, pixel_order=neopixel.RGB)

# while True:
pixels.fill((0, 255, 0))  # 赤
pixels.show()
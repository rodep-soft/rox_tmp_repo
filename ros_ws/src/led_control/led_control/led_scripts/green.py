# import time
import board
import neopixel

pixels = neopixel.NeoPixel(board.D23, 6, brightness=0.5, auto_write=False, pixel_order=neopixel.RGB)

pixels.fill((255, 0, 0))
pixels.show()
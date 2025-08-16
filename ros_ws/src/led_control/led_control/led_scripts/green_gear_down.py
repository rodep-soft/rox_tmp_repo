import fcntl
import time

import board
import neopixel

# # while True:
# pixels.fill((0, 255, 0))  # 赤
# pixels.show()

LOCK_FILE = "/tmp/led.lock"


def main():
    with open(LOCK_FILE, "w") as lockfile:
        fcntl.flock(lockfile, fcntl.LOCK_EX)
        try:
            pixels = neopixel.NeoPixel(
                board.D23, 6, brightness=0.3, auto_write=False, pixel_order=neopixel.RGB
            )
            pixels.fill((255, 0, 0))
            pixels.show()
            while True:
                time.sleep(1)
        finally:
            pixels.fill((0, 0, 0))
            pixels.show()
            fcntl.flock(lockfile, fcntl.LOCK_UN)


if __name__ == "__main__":
    main()

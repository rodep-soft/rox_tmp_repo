import gpiozero as gpio
from time import sleep

class Injection:
    __relay_pin = 17

    def __init__(self):
        self.relay_ = gpio.DigitalOutputDevice(Injection.__relay_pin)

    
    def relay(self, cycles=None):
        count = 0
        while cycles is None or count < cycles:
            self.relay_.on()
            print("start relay\n")
            sleep(2)  # wait for 2 seconds

            self.relay_.off()
            print("stop relay\n")
            sleep(2)  # wait for 2 seconds
            
            count += 1

            print(f"count = {count}")

    def finish(self):
        self.relay_.close()


def test_injection():
    inj = Injection()

    try:
        inj.relay(cycles=5)

    except KeyboardInterrupt:
        print("\nテストが中断")
    
    finally:
        inj.finish()
        print("リソースを解放")


# メイン実行
if __name__ == "__main__":
    test_injection()
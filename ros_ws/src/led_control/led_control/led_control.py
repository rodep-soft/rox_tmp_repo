from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import board
import neopixel

class LedControlNode(Node):
    def __init__(self):
        super().__init__("led_control_node")

        self.subscription = self.create_subscription(
            String,
            "/mode",
            self.mode_callback,
            10
        )

        # NeoPixelの初期化
        pixel_pin = board.D23

        num_pixels = 6

        ORDER = neopixel.GRB

        pixels = neopixel.NeoPixel(
            pixel_pin, num_pixels, brightness=1.0, auto_write=False, pixel_order=ORDER
        )

        self.pixels = pixels

        # 初期化テスト - 起動時に赤く点灯
        try:
            self.get_logger().info("LED初期化テスト開始")
            self.pixels.fill((255, 0, 0))
            self.pixels.show()
            self.get_logger().info("LED初期化完了")
        except Exception as e:
            self.get_logger().error(f"LED初期化エラー: {e}")

    def mode_callback(self, msg):
        mode = msg.data
        self.get_logger().info(f"モード切替: {mode}")
        
        try:
            if mode == "STOP":
                self.pixels.fill((255, 0, 0))  # 赤
                self.pixels.show()
                self.get_logger().info("LED: 赤色(STOP)")
            elif mode == "JOY":
                self.pixels.fill((0, 255, 0))  # 緑
                self.pixels.show()
                self.get_logger().info("LED: 緑色(JOY)")
            elif mode == "DPAD":
                self.pixels.fill((0, 0, 255))  # 青
                self.pixels.show()
                self.get_logger().info("LED: 青色(DPAD)")
            elif mode == "LINETRACE":
                self.pixels.fill((255, 255, 255))  # 白
                self.pixels.show()
                self.get_logger().info("LED: 白色(LINETRACE)")
            else:
                self.pixels.fill((0, 0, 0))  # 消灯
                self.pixels.show()
                self.get_logger().info("LED: 消灯")
        except Exception as e:
            self.get_logger().error(f"LED制御エラー: {e}")  


def main(args=None):
    rclpy.init(args=args)

    node = LedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


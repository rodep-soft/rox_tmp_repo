from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class LedControlNode(Node):
    def __init__(self):
        super().__init__("led_control_node")

        # LED スクリプトのパスを取得
        self.script_dir = os.path.join(os.path.dirname(__file__), "led_scripts")

        self.subscription = self.create_subscription(
            String,
            "/mode",
            self.mode_callback,
            10
        )

        # 初期化テスト - 起動時に赤く点灯
        try:
            self.get_logger().info("LED初期化テスト開始")
            # self.pixels.fill((255, 0, 0))
            # self.pixels.show()
            red_script = os.path.join(self.script_dir, "red.py")
            subprocess.Popen(["python3.11", red_script])
            self.get_logger().info("LED初期化完了")
        except Exception as e:
            self.get_logger().error(f"LED初期化エラー: {e}")

    def mode_callback(self, msg):
        mode = msg.data
        self.get_logger().info(f"モード切替: {mode}")
        
        try:
            if mode == "STOP":
                red_script = os.path.join(self.script_dir, "red.py")
                subprocess.Popen(["python3.11", red_script])
                self.get_logger().info("LED: 赤色(STOP)")
            elif mode == "JOY":
                green_script = os.path.join(self.script_dir, "green.py")
                subprocess.Popen(["python3.11", green_script])
                self.get_logger().info("LED: 緑色(JOY)")
            elif mode == "DPAD":
                blue_script = os.path.join(self.script_dir, "blue.py")
                subprocess.Popen(["python3.11", blue_script])
                self.get_logger().info("LED: 青色(DPAD)")
            elif mode == "LINETRACE":
                white_script = os.path.join(self.script_dir, "white.py")
                subprocess.Popen(["python3.11", white_script])
                self.get_logger().info("LED: 白色(LINETRACE)")
            else:
                off_script = os.path.join(self.script_dir, "off.py")
                subprocess.Popen(["python3.11", off_script])
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


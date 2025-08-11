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

        self.current_process = None

        self.driver_mode_subscription = self.create_subscription(
            String,
            "/mode",
            self.driver_mode_callback,
            10
        )



        # self.lifting_mode_subscription = self.create_subscription(
        #     String,
        #     "/lifting_mode",
        #     self.lifting_mode_callback,
        #     10
        # )

        # self.current_color = 'red'

        # # 初期化テスト - 起動時に赤く点灯
        # try:
        #     self.get_logger().info("LED初期化テスト開始")
        #     # self.pixels.fill((255, 0, 0))
        #     # self.pixels.show()
        #     red_script = os.path.join(self.script_dir, "red.py")
        #     subprocess.Popen(["python3.11", red_script])
        #     self.get_logger().info("LED初期化完了")
        # except Exception as e:
        #     self.get_logger().error(f"LED初期化エラー: {e}")

    def start_led_script(self, script_name):

        if self.current_process is not None:
            # self.get_logger().info("LED scriptを停止します")
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                # self.get_logger().warn("終了遅延のため強制終了")
                self.current_process.kill()
                self.current_process.wait()
            self.current_process = None

        script_path = os.path.join(self.script_dir, script_name)
        # self.get_logger().info(f"LED script開始: {script_name}")
        self.current_process = subprocess.Popen(["python3.11", script_path])

    def driver_mode_callback(self, msg):
        mode = msg.data
        # self.get_logger().info(f"モード切替: {mode}")
        
        # try:
        if mode == "STOP":
            self.start_led_script("red.py")
        elif mode == "JOY":
            self.start_led_script("green.py")
        elif mode == "DPAD":
            self.start_led_script("blue.py")
        elif mode == "LINETRACE":
            self.start_led_script("white.py")
        else:
            self.start_led_script("off.py")
        # except Exception as e:
        #     self.get_logger().error(f"LED制御エラー: {e}")  

    # def lifting_mode_callback(self, msg):
    #     mode = msg.data

    #     if mode == 'INIT':
        
    #     elif mode == 'TO_MAX':
        
    #     elif mode == 'RETURN_TO_MIN'


def main(args=None):
    rclpy.init(args=args)

    node = LedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


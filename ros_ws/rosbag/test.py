import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import rclpy.serialization
from geometry_msgs.msg import Twist
import rosidl_runtime_py.utilities

def main():
    db_path = input("データベースファイルのパスを入力してにゃ: ").strip()

    # /cmd_vel の型を取得
    msg_type = rosidl_runtime_py.utilities.get_message("geometry_msgs/msg/Twist")

    # DB接続
    conn = sqlite3.connect(db_path)
    df = pd.read_sql_query(
        "SELECT timestamp, data FROM messages WHERE topic_id = (SELECT id FROM topics WHERE name='/cmd_vel')",
        conn
    )

    timestamps = []
    linear_x = []
    linear_y = []
    angular_z = []

    for _, row in df.iterrows():
        twist_msg = rclpy.serialization.deserialize_message(row["data"], msg_type)
        timestamps.append(row["timestamp"] / 1e9)  # 秒に変換
        linear_x.append(twist_msg.linear.x)
        linear_y.append(twist_msg.linear.y)
        angular_z.append(twist_msg.angular.z)

    conn.close()

    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, linear_x, label="Linear X")
    plt.plot(timestamps, linear_y, label="Linear Y")
    plt.plot(timestamps, angular_z, label="Angular Z")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity")
    plt.legend()
    plt.title("/cmd_vel Data")

    plt.savefig("cmd_vel_plot.png")
    print("グラフを cmd_vel_plot.png に保存したにゃ 🐾")

if __name__ == "__main__":
    main()


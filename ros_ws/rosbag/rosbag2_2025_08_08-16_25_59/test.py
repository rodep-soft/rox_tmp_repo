import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import rclpy.serialization
from geometry_msgs.msg import Twist
import rosidl_runtime_py.utilities

# DBファイルのパス（カレントにある場合）
db_path = "rosbag2_2025_08_08-16_25_59_0.db3"

# /cmd_vel の型を取得
msg_type = rosidl_runtime_py.utilities.get_message("geometry_msgs/msg/Twist")

# DB接続
conn = sqlite3.connect(db_path)
df = pd.read_sql_query("SELECT timestamp, data FROM messages WHERE topic_id = (SELECT id FROM topics WHERE name='/cmd_vel')", conn)

# デシリアライズして DataFrame化
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

# グラフ描画
plt.figure(figsize=(10, 6))
plt.plot(timestamps, linear_x, label="Linear X")
plt.plot(timestamps, linear_y, label="Linear Y")
plt.plot(timestamps, angular_z, label="Angular Z")
plt.xlabel("Time [s]")
plt.ylabel("Velocity")
plt.legend()
plt.title("/cmd_vel Data")

# 画像として保存
plt.savefig("cmd_vel_plot.png")
print("グラフを cmd_vel_plot.png に保存したにゃ 🐾")


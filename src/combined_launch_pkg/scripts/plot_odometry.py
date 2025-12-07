import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
import os
from nav_msgs.msg import Odometry
import rclpy.serialization

# Path to your bag folder
bag_path = os.path.expanduser("~/ros2_ws/src/combined_launch_pkg/my_bag")

# Create storage and reader options
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions('', '')  # no conversion

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = {}

x_positions = []
y_positions = []
timestamps = []

while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == '/robot_gt':
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        x_positions.append(msg.pose.pose.position.x)
        y_positions.append(msg.pose.pose.position.y)
        timestamps.append(t / 1e9)  # convert nanoseconds to seconds

# Plot
plt.figure()
plt.plot(x_positions, y_positions, label='/odom path')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Robot Odometry Position')
plt.legend()
plt.grid()
plt.show()

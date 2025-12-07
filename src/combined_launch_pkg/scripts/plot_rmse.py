import rosbag2_py
from nav_msgs.msg import Odometry
import rclpy.serialization
import numpy as np
import os
import matplotlib.pyplot as plt

# Adjust this to your bag path and storage type
bag_path = os.path.expanduser('~/ros2_ws/src/combined_launch_pkg/my_bag')
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

odom_data = []
kalman_data = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/robot_gt':
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        odom_data.append({
            'time': t,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        })
    elif topic == '/robot_estimated_odometry':
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        kalman_data.append({
            'time': t,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        })

# Convert to numpy arrays for easier handling
odom_data = sorted(odom_data, key=lambda x: x['time'])
kalman_data = sorted(kalman_data, key=lambda x: x['time'])

# For simplicity, truncate to shortest length
N = min(len(odom_data), len(kalman_data))
#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy.serialization
import rosbag2_py
from nav_msgs.msg import Odometry

def main():
    # Adjust this path to where your rosbag folder is
    bag_path = os.path.expanduser('~/ros2_ws/src/combined_launch_pkg/my_bag')

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    odom_data = []
    kalman_data = []

    # Read messages
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/robot_gt':
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            odom_data.append({
                'time': t,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            })
        elif topic == '/robot_estimated_odometry':
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            kalman_data.append({
                'time': t,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            })

    # Sort by time (just in case)
    odom_data = sorted(odom_data, key=lambda x: x['time'])
    kalman_data = sorted(kalman_data, key=lambda x: x['time'])

    # Truncate to equal lengths for simple alignment
    N = min(len(odom_data), len(kalman_data))
    if N == 0:
        print("No data found in topics!")
        return

    odom_positions = np.array([[d['x'], d['y']] for d in odom_data[:N]])
    kalman_positions = np.array([[d['x'], d['y']] for d in kalman_data[:N]])
    times = np.array([d['time'] for d in odom_data[:N]])  # Use odom times as reference

    # Compute pointwise Euclidean error
    errors = np.linalg.norm(odom_positions - kalman_positions, axis=1)

    # Compute overall RMSE
    rmse = np.sqrt(np.mean(errors**2))
    print(f"Overall RMSE: {rmse:.4f} meters")

    # Analyze convergence speed and stability
    threshold = 0.1  # meters, define your convergence threshold

    # Find first sample index where error < threshold
    converged_indices = np.where(errors < threshold)[0]
    if len(converged_indices) > 0:
        convergence_index = converged_indices[0]
        convergence_time = (times[convergence_index] - times[0]) / 1e9  # convert ns to seconds
        print(f"Converged at sample {convergence_index} (time: {convergence_time:.2f} s)")

        stable_errors = errors[convergence_index:]
        mean_error = np.mean(stable_errors)
        std_error = np.std(stable_errors)
        print(f"Mean error after convergence: {mean_error:.4f} m")
        print(f"Error std deviation after convergence: {std_error:.4f} m")
    else:
        convergence_index = None
        print("Did not converge within the threshold.")

    # Plot error over time with convergence mark
    plt.figure(figsize=(10, 5))
    time_sec = (times - times[0]) / 1e9  # convert nanoseconds to seconds

    plt.plot(time_sec, errors, label='Position error (m)')
    plt.axhline(y=threshold, color='r', linestyle='--', label=f'Convergence threshold ({threshold} m)')
    if convergence_index is not None:
        plt.axvline(x=time_sec[convergence_index], color='g', linestyle='--', label='Convergence time')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Position error between /odom and /kalman_odom over time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

odom_positions = np.array([[1,2], [2,3], [3,4]])
kalman_positions = np.array([[1.1,2.1], [1.9,3.1], [3.05,3.95]])

print("odom_positions shape:", odom_positions.shape)  # should be (N,2)
print("kalman_positions shape:", kalman_positions.shape)  # should be (N,2)

rmse = np.sqrt(np.mean(np.sum((odom_positions - kalman_positions)**2, axis=1)))
print("RMSE:", rmse)

errors = np.linalg.norm(odom_positions - kalman_positions, axis=1)

threshold = 0.1  # meters

if np.any(errors < threshold):
    convergence_index = np.argmax(errors < threshold)
    print(f"Converged at sample {convergence_index}")
    stable_errors = errors[convergence_index:]
    print(f"Mean error after convergence: {np.mean(stable_errors):.4f} m")
    print(f"Error std dev after convergence: {np.std(stable_errors):.4f} m")
else:
    convergence_index = None
    print("Convergence threshold not reached.")

plt.plot(errors, label='Position error')
plt.axhline(y=threshold, color='r', linestyle='--', label='Threshold')
if convergence_index is not None:
    plt.axvline(x=convergence_index, color='g', linestyle='--', label='Convergence point')
plt.xlabel('Sample index (time)')
plt.ylabel('Position error (m)')
plt.legend()
plt.grid(True)
plt.title('Position error over time')
plt.show()

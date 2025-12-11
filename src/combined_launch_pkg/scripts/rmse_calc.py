import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
import os
from nav_msgs.msg import Odometry
import rclpy.serialization
from scipy.spatial import cKDTree # Used for efficient nearest-neighbor lookup (synchronization)
from scipy.optimize import curve_fit # Not needed for RMSE, but good to keep if you plan to do curve fitting
from scipy.interpolate import interp1d # Used for accurate synchronization


def load_odometry(bag_path, topic, storage_id="mcap"):
    """Loads Odometry data (x, y, t) from a ROS 2 bag file."""
    # ... (Your existing function remains the same) ...
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return np.array([]), np.array([])

    xs, ys, times = [], [], []

    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name == topic:
            try:
                msg = rclpy.serialization.deserialize_message(data, Odometry)
                p = msg.pose.pose.position
                xs.append(p.x)
                ys.append(p.y)
                times.append(t / 1e9)  # nanoseconds → seconds
            except Exception as e:
                # print(f"Error deserializing message: {e}")
                continue # Skip corrupt messages

    return np.array(times), np.vstack([xs, ys]).T


def calculate_rmse(t_ref, p_ref, t_run, p_run):
    """
    Calculates the Root Mean Square Error (RMSE) between two trajectories.

    1. Synchronizes p_run to t_ref using linear interpolation.
    2. Computes the squared error.
    3. Calculates the RMSE.
    """
    if len(t_ref) == 0 or len(t_run) == 0:
        return np.nan

    # 1. Synchronize p_run to t_ref using linear interpolation
    # Create an interpolation function for the run positions (x and y separately)
    # The 'bounds_error=False' and 'fill_value="extrapolate"' handles points 
    # outside the time range by using the nearest available value (a common practice).
    
    # Interpolate X-coordinates
    f_x = interp1d(t_run, p_run[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
    x_run_sync = f_x(t_ref)
    
    # Interpolate Y-coordinates
    f_y = interp1d(t_run, p_run[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
    y_run_sync = f_y(t_ref)
    
    p_run_sync = np.vstack([x_run_sync, y_run_sync]).T
    
    # 2. Calculate the difference (error)
    # The error is the Euclidean distance between the reference and synchronized run positions
    
    # The squared error for each point: (ref_x - sync_x)^2 + (ref_y - sync_y)^2
    squared_error = np.sum((p_ref - p_run_sync)**2, axis=1)
    
    # 3. Calculate the RMSE
    # RMSE = sqrt( Mean(Squared Error) )
    mean_squared_error = np.mean(squared_error)
    rmse = np.sqrt(mean_squared_error)
    
    return rmse, p_ref, p_run_sync # Return the synchronized trajectories for plotting



if __name__ == "__main__":
    bag_path = os.path.expanduser('~/ros2_ws/src/combined_launch_pkg/my_bag')
    
    # Load data
    t_ref, p_ref = load_odometry(bag_path, topic='/robot_gt')
    t_run, p_run = load_odometry(bag_path, topic='/robot_estimated_odometry')

    print(f"Reference points loaded: {len(p_ref)}")
    print(f"Run points loaded: {len(p_run)}")

    if len(p_ref) > 0 and len(p_run) > 0:
        # Calculate RMSE
        rmse_value, p_ref_sync, p_run_sync = calculate_rmse(t_ref, p_ref, t_run, p_run)
        
        print(f"\n**Calculated RMSE:** **{rmse_value:.4f}** meters")

        # Optional: Plot the trajectories and error
        plt.figure(figsize=(10, 8))
        
        # Plot Trajectories
        plt.plot(p_ref[:, 0], p_ref[:, 1], 'k--', label='Reference (Ground Truth)')
        plt.plot(p_run[:, 0], p_run[:, 1], 'r-', alpha=0.6, label='Estimated (Run)')
        
        # Plot Synchronization/Error Lines (optional, for visual verification)
        # We can draw lines between the synchronized points to visualize the error
        for i in range(0, len(p_ref_sync), len(p_ref_sync)//20): # Only plot a few for clarity
            plt.plot([p_ref_sync[i, 0], p_run_sync[i, 0]], 
                     [p_ref_sync[i, 1], p_run_sync[i, 1]], 'b-', alpha=0.3, linewidth=0.5)

        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.title(f'Trajectory Comparison (RMSE: {rmse_value:.4f} m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') # Ensure scale is accurate for comparison
        plt.show()
    else:
        print("\n Cannot calculate RMSE: One or both odometry topics contain no data.")
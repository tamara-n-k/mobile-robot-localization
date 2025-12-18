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



# (Your existing imports, load_odometry, and __main__ block remain the same)
# ...
# from scipy.interpolate import interp1d # Used for accurate synchronization 

# --- New/Modified Analysis Functions ---

def calculate_errors_and_rmse(t_ref, p_ref, t_run, p_run):
    """
    MODIFIED: Calculates the full time-series of positional errors and the overall RMSE.

    Returns:
        rmse (float): The overall RMSE value.
        errors_time_series (np.array): Positional error at each t_ref point.
        p_ref_sync (np.array): The reference path (p_ref).
        p_run_sync (np.array): The run path synchronized to t_ref.
    """
    if len(t_ref) == 0 or len(t_run) == 0:
        return np.nan, np.array([]), np.array([]), np.array([])

    # 1. Synchronize p_run to t_ref using linear interpolation (X and Y)
    f_x = interp1d(t_run, p_run[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
    f_y = interp1d(t_run, p_run[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
    
    x_run_sync = f_x(t_ref)
    y_run_sync = f_y(t_ref)
    
    p_run_sync = np.vstack([x_run_sync, y_run_sync]).T
    
    # 2. Calculate the difference (error) - Euclidean distance
    # Positional Error (at each time step): sqrt((ref_x - sync_x)^2 + (ref_y - sync_y)^2)
    squared_error = np.sum((p_ref - p_run_sync)**2, axis=1)
    positional_errors = np.sqrt(squared_error) # The time-series error
    
    # 3. Calculate the overall RMSE
    mean_squared_error = np.mean(squared_error)
    rmse = np.sqrt(mean_squared_error)
    
    return rmse, positional_errors, p_ref, p_run_sync


def analyze_convergence(t_ref, errors_time_series, convergence_threshold=0.15):
    """
    Measures the time it takes for the error to drop and stay below a threshold.

    Args:
        t_ref (np.array): Time stamps (in seconds).
        errors_time_series (np.array): Positional error at each time step.
        convergence_threshold (float): The error level (in meters) considered 'converged'.

    Returns:
        float: The time elapsed (in seconds) until convergence, or NaN if never converged.
    """
    # Create time elapsed from start
    t_elapsed = t_ref - t_ref[0]

    # Find the index where the error drops below the threshold
    initial_drop_idx = np.where(errors_time_series <= convergence_threshold)[0]
    
    if len(initial_drop_idx) == 0:
        return np.nan, np.nan, t_elapsed # Never converged

    # The first time the error drops below the threshold
    start_search_idx = initial_drop_idx[0]

    # Check the remaining time series from that point
    # We want to find the LAST point where the error is ABOVE the threshold
    # The time to convergence is immediately after that last spike.
    
    last_overshoot_idx = start_search_idx - 1 # Start checking from before the first drop
    
    # Look for any points *after* the initial drop where the error went above the threshold
    overshoot_indices = np.where(errors_time_series[start_search_idx:] > convergence_threshold)[0]
    
    if len(overshoot_indices) > 0:
        # If there were overshoots, find the last one (relative to the start_search_idx)
        last_overshoot_idx = start_search_idx + overshoot_indices[-1]
        
    # Convergence time is the time stamp immediately after the last overshoot (or the first drop)
    convergence_idx = last_overshoot_idx + 1 
    
    if convergence_idx >= len(t_ref):
         # If convergence point is outside the run time
        return np.nan, convergence_threshold, t_elapsed # Didn't fully converge

    convergence_time = t_elapsed[convergence_idx]
    
    return convergence_time, convergence_threshold, t_elapsed


def analyze_stability(t_elapsed, errors_time_series, convergence_time):
    """
    Analyzes stability metrics *after* the system has converged.

    Args:
        t_elapsed (np.array): Time elapsed from start (in seconds).
        errors_time_series (np.array): Positional error at each time step.
        convergence_time (float): The time point when convergence was reached.

    Returns:
        dict: Stability metrics (Mean Error, Std Dev Error, Max Error).
    """
    if np.isnan(convergence_time):
        print("Warning: Convergence time is NaN. Analyzing stability across the full run.")
        stable_errors = errors_time_series
    else:
        # Select error points that occur AFTER the convergence time
        post_convergence_indices = t_elapsed >= convergence_time
        stable_errors = errors_time_series[post_convergence_indices]

    if len(stable_errors) < 10: # Minimum data points for meaningful statistics
        print("Warning: Insufficient data points for stability analysis after convergence.")
        return {
            'Mean Error (m)': np.nan,
            'Std Dev Error (m)': np.nan,
            'Max Error (m)': np.nan
        }

    return {
        'Mean Error (m)': np.mean(stable_errors),
        'Std Dev Error (m)': np.std(stable_errors), # Low std dev = High stability
        'Max Error (m)': np.max(stable_errors) # Low max error = Bounded error
    }

# --- Main Execution Block Update ---

if __name__ == "__main__":
    bag_path = os.path.expanduser('~/ros2_ws/src/combined_launch_pkg/pn_0.01')
    CONV_THRESHOLD = 0.15 # Set your convergence error tolerance (e.g., 15 cm)
    
    # Load data
    t_ref, p_ref = load_odometry(bag_path, topic='/robot_gt')
    t_run, p_run = load_odometry(bag_path, topic='/robot_estimated_odometry')

    print(f"Reference points loaded: {len(p_ref)}")
    print(f"Run points loaded: {len(p_run)}")

    if len(p_ref) > 0 and len(p_run) > 0:
        # 1. Calculate Errors and Overall RMSE
        rmse_value, errors_ts, p_ref_sync, p_run_sync = calculate_errors_and_rmse(t_ref, p_ref, t_run, p_run)
        
        # 2. Analyze Convergence
        conv_time, threshold, t_elapsed = analyze_convergence(t_ref, errors_ts, CONV_THRESHOLD)
        
        # 3. Analyze Stability
        stability_metrics = analyze_stability(t_elapsed, errors_ts, conv_time)

        # 4. Print Key Results
        print("\n--- Summary of Estimation Performance ---")
        print(f"**Overall Positional RMSE:** **{rmse_value:.4f}** meters")
        
        # Convergence Results
        print("\n--- Convergence Analysis ---")
        if np.isnan(conv_time):
            print(f"The estimate did **NOT** converge below the threshold of {threshold} m.")
        else:
            print(f"**Convergence Time (T_conv):** **{conv_time:.3f}** seconds")
            print(f"Threshold used: {threshold} meters")

        # Stability Results
        print("\n--- Stability Analysis (Post-Convergence) ---")
        if not np.isnan(stability_metrics['Std Dev Error (m)']):
            for key, val in stability_metrics.items():
                print(f"**{key}:** **{val:.4f}**")
            print("\n*Lower 'Std Dev Error' indicates higher estimation stability.*")
        else:
            print("Stability metrics could not be calculated.")

        # 5. Visualization
        plt.figure(figsize=(10, 12))
        
        # Plot 1: Trajectory Comparison (Top)
        plt.subplot(2, 1, 1)
        plt.plot(p_ref[:, 0], p_ref[:, 1], 'k--', label='Reference (Ground Truth)')
        plt.plot(p_run[:, 0], p_run[:, 1], 'r-', alpha=0.6, label='Estimated (Run)')
        plt.title(f'Trajectory Comparison (RMSE: {rmse_value:.4f} m)')
        plt.xlabel('X position (m)'); plt.ylabel('Y position (m)')
        plt.legend(); plt.grid(True); plt.axis('equal')
        
        # Plot 2: Positional Error Time Series (Bottom) 
        plt.subplot(2, 1, 2)
        plt.plot(t_elapsed, errors_ts, 'g-', label='Positional Error ($\sqrt{\Delta x^2 + \Delta y^2}$)', linewidth=1)
        plt.axhline(rmse_value, color='black', linestyle=':', label=f'Overall RMSE: {rmse_value:.4f} m')
        
        # Mark Convergence on the plot
        plt.axhline(threshold, color='red', linestyle='--', label=f'Convergence Threshold ({threshold} m)')
        if not np.isnan(conv_time):
            plt.axvline(conv_time, color='purple', linestyle='-', linewidth=2, label=f'Convergence Time ({conv_time:.3f} s)')
            
        plt.title('Convergence and Stability Analysis')
        plt.xlabel('Time Elapsed (s)'); plt.ylabel('Positional Error (m)')
        plt.legend(); plt.grid(True)
        
        plt.tight_layout()
        plt.show()

    else:
        print("\n Cannot calculate RMSE: One or both odometry topics contain no data.")
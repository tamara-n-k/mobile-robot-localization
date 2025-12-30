import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import matplotlib
matplotlib.use('TkAgg')

import csv
import os


class OdomConvergencePlot(Node):

    def __init__(self):
        super().__init__('odom_convergence_plot')

        self.T_END = 30.0   # seconds

        self.create_subscription(Odometry, '/robot_estimated_odometry', self.odom_cb, 10)
        self.create_subscription(Odometry, '/robot_gt', self.gt_cb, 10)

        self.odom = None
        self.gt = None

        self.pos_err = []
        self.err_vec = []      # stores [dx, dy, dz]
        self.rmse = []
        self.WINDOW = 50      # samples (~5s at 10 Hz)
        self.yaw_err = []
        self.yaw_err_buf = []      # radians

        self.time = []
        self.t0 = None   # reference time (seconds)
        self.last_time = None

        # Stabilization parameters
        self.RMSE_THRESH = 0.25   # meters
        self.STABLE_TIME = 5.0    # seconds
        self.stable_duration = 0.0
        self.stable = False
        self.t_stable = None

        self.YAW_RMSE_THRESH = 5.0  # degrees
        self.yaw_stable_duration = 0.0
        self.yaw_stable = False
        self.t_yaw_stable = None

        self.tracking_rmse_buf = []     # RMSE after stabilization
        self.tracking_line = None

        # matplotlib interactive mode
        plt.ion()

        # Position RMSE plot
        self.fig_pos, self.ax_pos = plt.subplots()
        self.fig_pos.suptitle("Odometry Position RMSE")
        self.l_pos, = self.ax_pos.plot([], [], label="Position RMSE [m]")
        self.ax_pos.set_xlabel("Time [s]")
        self.ax_pos.set_ylabel("RMSE [m]")
        self.ax_pos.axhline(self.RMSE_THRESH, color='red', linestyle=':', label='RMSE threshold')
        self.ax_pos.legend()
        self.ax_pos.grid(True)

        # Yaw RMSE plot
        self.fig_yaw, self.ax_yaw = plt.subplots()
        self.fig_yaw.suptitle("Odometry Yaw RMSE")
        self.l_yaw, = self.ax_yaw.plot([], [], label="Yaw RMSE [deg]")
        self.ax_yaw.set_xlabel("Time [s]")
        self.ax_yaw.set_ylabel("RMSE [deg]")
        self.ax_yaw.legend()
        self.ax_yaw.grid(True)

        # open plot immediately
        self.fig_pos.canvas.draw()
        self.fig_pos.canvas.flush_events()
        plt.pause(0.001)

        # Timer only for plot refresh
        self.timer = self.create_timer(0.1, self.update_plot)

        self.csv_file_path = os.path.join(os.getcwd(), 'stabilization_results.csv')
        file_exists = os.path.isfile(self.csv_file_path)

        self.csv_file = open(self.csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        if not file_exists:
            self.csv_writer.writerow([
                "Stabilization Time [s]", 
                "Position RMSE at Stabilization [m]",
                "Yaw Stabilization Time [s]", 
                "Yaw RMSE at Stabilization [deg]", 
                "Final Tracking RMSE [m]"
            ])
        self.save_dir = os.path.join(os.getcwd(), "plots")
        os.makedirs(self.save_dir, exist_ok=True)


    def odom_cb(self, msg: Odometry):
        # Store odom message and process only if gt is available
        self.odom = msg
        if self.gt is None:
            return

        # Extract timestamps (seconds)
        t_msg = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9

        # Initialize time zero
        if self.t0 is None:
            self.t0 = t_msg
            self.last_time = 0.0
            self.get_logger().info(f"t0 set at message timestamp {self.t0:.3f} seconds")
            return

        # Calculate current relative time and delta time
        t = t_msg - self.t0
        dt = t - self.last_time
        if dt <= 0:
            # Ignore non-monotonic or duplicate timestamps
            return
        self.last_time = t

        # Compute position and yaw error
        pos_rmse, yaw_rmse_deg = self.compute_error()

        # Append results to buffers
        self.time.append(t)
        self.pos_err.append(pos_rmse)
        self.yaw_err.append(yaw_rmse_deg)

        # Stabilization logic for position
        if not self.stable:
            if pos_rmse < self.RMSE_THRESH:
                self.stable_duration += dt
                if self.stable_duration >= self.STABLE_TIME:
                    self.stable = True
                    self.t_stable = t
                    self.get_logger().info(
                        f"Odometry stabilized at t = {t:.2f} s (RMSE < {self.RMSE_THRESH})"
                        f"Odometry RMSE is {pos_rmse} m"
                    )
            else:
                self.stable_duration = 0.0

        # Annotate stabilization line once
        if self.stable and not hasattr(self, '_annotated'):
            self.ax_pos.axvline(
                self.t_stable,
                color='green',
                linestyle='--',
                label='Stable'
            )
            self.ax_pos.legend()
            self._annotated = True
        self.pos_stab_time = self.t_stable
        self.pos_stab_rmse = pos_rmse

        # Stabilization logic for yaw
        if not self.yaw_stable:
            if yaw_rmse_deg < self.YAW_RMSE_THRESH:
                self.yaw_stable_duration += dt
                if self.yaw_stable_duration >= self.STABLE_TIME:
                    self.yaw_stable = True
                    self.t_yaw_stable = t
                    self.get_logger().info(
                        f"Yaw stabilized at t = {t:.2f} s (RMSE < {self.YAW_RMSE_THRESH} deg), yaw rmse: {yaw_rmse_deg}"
                    )
            else:
                self.yaw_stable_duration = 0.0

        if self.yaw_stable and not hasattr(self, '_yaw_annotated'):
            self.ax_yaw.axvline(
                self.t_yaw_stable,
                color='green',
                linestyle='--',
                label='Yaw stable'
            )
            self.ax_yaw.legend()
            self._yaw_annotated = True
        self.yaw_stab_time = self.t_yaw_stable
        self.yaw_stab_rmse = yaw_rmse_deg

        # Track RMSE after stabilization
        if self.stable:
            self.tracking_rmse_buf.append(pos_rmse)

        if self.stable and len(self.tracking_rmse_buf) > 0:
            tracking_rmse = np.mean(self.tracking_rmse_buf)
            if self.tracking_line is None:
                self.tracking_line = self.ax_pos.axhline(
                    tracking_rmse,
                    linestyle=':',
                    color='purple',
                    label='Tracking RMSE (mean)'
                )
                self.ax_pos.legend()
            else:
                self.tracking_line.set_ydata([tracking_rmse, tracking_rmse])
        

        if t >= self.T_END:
            if self.stable and len(self.tracking_rmse_buf) > 0:
                final_tracking_rmse = np.mean(self.tracking_rmse_buf)
                self.get_logger().info(
                    f"Final tracking RMSE = {final_tracking_rmse:.3f} m"
                )
            else:
                final_tracking_rmse = None

            self.fig_pos.savefig(
            os.path.join(self.save_dir, "position_rmse.png"),
            dpi=300,
            bbox_inches="tight"
            )

            self.fig_yaw.savefig(
            os.path.join(self.save_dir, "yaw_rmse.png"),
            dpi=300,
            bbox_inches="tight"
            )



            # Write a final row to CSV
            self.csv_writer.writerow([
                getattr(self, 'pos_stab_time', None),
                getattr(self, 'pos_stab_rmse', None),
                getattr(self, 'yaw_stab_time', None),
                getattr(self, 'yaw_stab_rmse', None),
                final_tracking_rmse
            ])

            self.csv_file.close()
            self.get_logger().info(f"Results written to {self.csv_file_path}")
            self.get_logger().info("Simulation finished.")
            rclpy.shutdown()


        
        

    def gt_cb(self, msg: Odometry):
        self.gt = msg

    def yaw_from_quat(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('xyz')[2]

    def compute_error(self):
        p1 = self.odom.pose.pose.position
        p2 = self.gt.pose.pose.position

        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z

        self.err_vec.append([dx, dy, dz])
        if len(self.err_vec) > self.WINDOW:
            self.err_vec.pop(0)

        err_arr = np.array(self.err_vec)
        pos_rmse = np.sqrt(np.mean(np.sum(err_arr ** 2, axis=1)))

        yaw1 = self.yaw_from_quat(self.odom.pose.pose.orientation)
        yaw2 = self.yaw_from_quat(self.gt.pose.pose.orientation)

        dyaw = np.arctan2(np.sin(yaw1 - yaw2), np.cos(yaw1 - yaw2))

        self.yaw_err_buf.append(dyaw)
        if len(self.yaw_err_buf) > self.WINDOW:
            self.yaw_err_buf.pop(0)

        yaw_rmse = np.sqrt(np.mean(np.array(self.yaw_err_buf) ** 2))
        yaw_rmse_deg = np.degrees(yaw_rmse)

        return pos_rmse, yaw_rmse_deg

    def update_plot(self):
        if not self.time:
            return

        # Update position RMSE plot
        self.l_pos.set_data(self.time, self.pos_err)
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()
        self.fig_pos.canvas.draw()
        self.fig_pos.canvas.flush_events()

        # Update yaw RMSE plot
        self.l_yaw.set_data(self.time, self.yaw_err)
        self.ax_yaw.relim()
        self.ax_yaw.autoscale_view()
        self.fig_yaw.canvas.draw()
        self.fig_yaw.canvas.flush_events()


def main():
    rclpy.init()
    node = OdomConvergencePlot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

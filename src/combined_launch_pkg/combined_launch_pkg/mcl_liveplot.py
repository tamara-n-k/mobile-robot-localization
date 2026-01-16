import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

import csv
import os


class MCLConvergencePlot(Node):

    def __init__(self):
        super().__init__('mcl_convergence_plot')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.T_END = 40.0          # seconds
        self.WINDOW = 50           # sliding RMSE window for live RMSE calc
        self.RMSE_THRESH = 0.25    # meters (for closeness threshold)
        self.STABLE_TIME = 5.0     # seconds (must be below RMSE_THRESH this long)

        self.LOCAL_STAB_WINDOW = 50      # samples ahead to average for local stability
        self.LOCAL_STAB_THRESH = 0.02    # meters (max allowed change for stability)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(
            Odometry,
            '/robot_estimated_odometry_mcl',
            self.odom_cb,
            10
        )
        self.create_subscription(
            Odometry,
            '/robot_gt',
            self.gt_cb,
            10
        )

        self.odom = None
        self.gt = None

        # -----------------------------
        # Buffers
        # -----------------------------
        self.time = []
        self.err_vec = []
        self.pos_rmse = []

        self.t0 = None
        self.last_time = None

        # -----------------------------
        # Stability
        # -----------------------------
        self.stable = False
        self.stable_duration = 0.0
        self.t_stable = None

        # -----------------------------
        # Convergence tracking (threshold)
        # -----------------------------
        self.below_thresh = False
        self.below_thresh_duration = 0.0
        self.t_below_thresh_start = None

        # -----------------------------
        # Convergence tracking (combined local stability)
        # -----------------------------
        self.converged = False
        self.t_converged = None

        # -----------------------------
        # Tracking steady-state RMSE
        # -----------------------------
        self.tracking_rmse_buf = []
        self.tracking_line = None

        # -----------------------------
        # Plot setup
        # -----------------------------
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.suptitle("MCL Position RMSE")

        self.l_rmse, = self.ax.plot([], [], label="Position RMSE [m]")
        self.ax.axhline(
            self.RMSE_THRESH,
            color='red',
            linestyle=':',
            label=f'RMSE Threshold ({self.RMSE_THRESH} m)'
        )

        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("RMSE [m]")
        self.ax.grid(True)
        self.ax.legend()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.timer = self.create_timer(0.1, self.update_plot)

        # -----------------------------
        # CSV logging
        # -----------------------------
        self.csv_path = os.path.join(os.getcwd(), 'mcl_convergence_results.csv')
        file_exists = os.path.isfile(self.csv_path)

        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        if not file_exists:
            self.csv_writer.writerow([
                "Convergence Time [s]",
                "RMSE at Convergence [m]",
                "Stability Time [s]",
                "Final Tracking RMSE [m]"
            ])


        self.save_dir = os.path.join(os.getcwd(), "plots")
        os.makedirs(self.save_dir, exist_ok=True)

    # --------------------------------------------------

    def gt_cb(self, msg: Odometry):
        self.gt = msg

    # --------------------------------------------------

    def odom_cb(self, msg: Odometry):
        self.odom = msg
        if self.gt is None:
            return

        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.t0 is None:
            self.t0 = t_msg
            self.last_time = 0.0
            self.get_logger().info("t0 initialized")
            return

        t = t_msg - self.t0
        dt = t - self.last_time
        if dt <= 0:
            return
        self.last_time = t

        rmse = self.compute_position_rmse()

        self.time.append(t)
        self.pos_rmse.append(rmse)

        # Stability detection
        if not self.stable:
            if rmse < self.RMSE_THRESH:
                self.stable_duration += dt
                if self.stable_duration >= self.STABLE_TIME:
                    self.stable = True
                    self.t_stable = t
                    self.get_logger().info(
                        f"Stability reached at t = {t:.2f} s (RMSE < {self.RMSE_THRESH})"
                    )
                    self.ax.axvline(
                        t,
                        color='green',
                        linestyle='--',
                        label='Stable'
                    )
                    self.ax.legend()
            else:
                self.stable_duration = 0.0

        # Step 1: Threshold + time stability check (for convergence)
        if rmse < self.RMSE_THRESH:
            if not self.below_thresh:
                self.below_thresh = True
                self.below_thresh_duration = 0.0
                self.t_below_thresh_start = t
            else:
                self.below_thresh_duration += dt
        else:
            self.below_thresh = False
            self.below_thresh_duration = 0.0
            self.t_below_thresh_start = None

        # Step 2: Local stability check for convergence
        if (self.below_thresh and
            self.below_thresh_duration >= self.STABLE_TIME and
            not self.converged and
            len(self.pos_rmse) > self.LOCAL_STAB_WINDOW * 2):

            i = len(self.pos_rmse) - 1 - self.LOCAL_STAB_WINDOW
            current_err = self.pos_rmse[i]
            future_avg = np.mean(self.pos_rmse[i + 1 : i + 1 + self.LOCAL_STAB_WINDOW])

            if abs(current_err - future_avg) <= self.LOCAL_STAB_THRESH:
                self.converged = True
                self.t_converged = self.time[i]
                self.rmse_at_convergence = current_err
                self.get_logger().info(
                    f"MCL converged at t = {self.t_converged:.2f} s (RMSE = {current_err:.3f} m)"
                )
                # self.ax.axvline(
                #     self.t_converged,
                #     color='blue',
                #     linestyle='--',
                #     label='Converged'
                # )
                self.ax.legend()

        # Track steady-state RMSE after convergence
        if self.converged:
            self.tracking_rmse_buf.append(rmse)

            mean_tracking_rmse = np.mean(self.tracking_rmse_buf)
            if self.tracking_line is None:
                self.tracking_line = self.ax.axhline(
                    mean_tracking_rmse,
                    color='purple',
                    linestyle=':',
                    label='Tracking RMSE (mean)'
                )
                self.ax.legend()
            else:
                self.tracking_line.set_ydata([mean_tracking_rmse, mean_tracking_rmse])

        # End of experiment
        if t >= self.T_END:
            final_tracking_rmse = (
                np.mean(self.tracking_rmse_buf)
                if self.tracking_rmse_buf else None
            )

            self.csv_writer.writerow([
                self.t_converged,
                self.rmse_at_convergence,
                self.t_stable,
                final_tracking_rmse
            ])

            self.csv_file.close()

            self.fig.savefig(
                os.path.join(self.save_dir, "mcl_position_rmse.png"),
                dpi=300,
                bbox_inches="tight"
            )

            self.get_logger().info("MCL evaluation finished.")
            rclpy.shutdown()

    def compute_position_rmse(self):
        p1 = self.odom.pose.pose.position
        p2 = self.gt.pose.pose.position

        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z

        self.err_vec.append([dx, dy, dz])
        if len(self.err_vec) > self.WINDOW:
            self.err_vec.pop(0)

        err_arr = np.array(self.err_vec)
        rmse = np.sqrt(np.mean(np.sum(err_arr ** 2, axis=1)))
        return rmse

    # --------------------------------------------------

    def update_plot(self):
        if not self.time:
            return

        self.l_rmse.set_data(self.time, self.pos_rmse)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = MCLConvergencePlot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

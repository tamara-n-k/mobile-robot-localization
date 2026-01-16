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
        self.T_END = 30.0          # seconds
        self.WINDOW = 50           # sliding RMSE window
        self.RMSE_THRESH = 0.25    # meters
        self.STABLE_TIME = 5.0     # seconds

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
        # Convergence tracking
        # -----------------------------
        self.stable = False
        self.stable_duration = 0.0
        self.t_stable = None

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
            label='RMSE threshold'
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
                "Stabilization Time [s]",
                "RMSE at Stabilization [m]",
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

        # -----------------------------
        # Convergence logic
        # -----------------------------
        if not self.stable:
            if rmse < self.RMSE_THRESH:
                self.stable_duration += dt
                if self.stable_duration >= self.STABLE_TIME:
                    self.stable = True
                    self.t_stable = t
                    self.get_logger().info(
                        f"MCL converged at t = {t:.2f} s (RMSE = {rmse:.3f} m)"
                    )
                    self.ax.axvline(
                        t,
                        color='green',
                        linestyle='--',
                        label='Converged'
                    )
                    self.ax.legend()
            else:
                self.stable_duration = 0.0

        # Track steady-state RMSE
        if self.stable:
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
                self.tracking_line.set_ydata(
                    [mean_tracking_rmse, mean_tracking_rmse]
                )

        # -----------------------------
        # End of experiment
        # -----------------------------
        if t >= self.T_END:
            final_tracking_rmse = (
                np.mean(self.tracking_rmse_buf)
                if self.tracking_rmse_buf else None
            )

            self.csv_writer.writerow([
                self.t_stable,
                self.pos_rmse[-1],
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

    # --------------------------------------------------

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

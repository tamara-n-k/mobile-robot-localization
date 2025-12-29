import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import matplotlib
matplotlib.use('TkAgg')

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
        self.yaw_rmse = []

        self.time = []
        self.t0 = None   # reference time (seconds)

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update_plot)


        self.RMSE_THRESH = 0.25   # meters
        self.STABLE_TIME = 5.0    # seconds

        self.stable_duration = 0.0
        self.stable = False
        self.t_stable = None
        self.last_time = None
        
        self.STAB_THRESHOLD = 0.25      # meters
        self.STAB_DURATION = 5.0        # seconds
        self.is_stable = False
        self.stable_start_time = None
        self.t_stab = None

        self.tracking_rmse_buf = []     # RMSE after stabilization
        self.tracking_line = None


        # matplotlib
        plt.ion() # enables interactive mode

        # Position RMSE plot
        self.fig_pos, self.ax_pos = plt.subplots()
        self.fig_pos.suptitle("Odometry Position RMSE")
        self.l_pos, = self.ax_pos.plot([], [], label="Position RMSE [m]")
        self.ax_pos.set_xlabel("Time [s]")
        self.ax_pos.set_ylabel("RMSE [m]")
        self.ax_pos.legend()
        self.ax_pos.grid(True)

        self.ax_pos.axhline(
        self.RMSE_THRESH,
        color='red',
        linestyle=':',
        label='RMSE threshold'
        )
        self.ax_pos.legend()


        # Yaw RMSE plot
        self.fig_yaw, self.ax_yaw = plt.subplots()
        self.fig_yaw.suptitle("Odometry Yaw RMSE")
        self.l_yaw, = self.ax_yaw.plot([], [], label="Yaw RMSE [deg]")
        self.ax_yaw.set_xlabel("Time [s]")
        self.ax_yaw.set_ylabel("RMSE [deg]")
        self.ax_yaw.legend()
        self.ax_yaw.grid(True)
        # yaw stabilization parameters
        self.YAW_RMSE_THRESH = 5.0  # threshold in radians
        self.yaw_stable = False
        self.yaw_stable_duration = 0.0
        self.t_yaw_stable = None

        self.last_time = None  # shared time step



    def odom_cb(self, msg):
        self.odom = msg

    def gt_cb(self, msg):
        self.gt = msg

    def yaw_from_quat(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('xyz')[2]

    def compute_error(self):
        # get poses
        p1 = self.odom.pose.pose.position
        p2 = self.gt.pose.pose.position

        # instantaneous position error
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z

        # add to buffer
        self.err_vec.append([dx, dy, dz]) 
        if len(self.err_vec) > self.WINDOW:
            self.err_vec.pop(0)

        # sliding RMSE
        err_arr = np.array(self.err_vec)
        pos_rmse = np.sqrt(np.mean(np.sum(err_arr**2, axis=1)))
    
        # get angles
        yaw1 = self.yaw_from_quat(self.odom.pose.pose.orientation)
        yaw2 = self.yaw_from_quat(self.gt.pose.pose.orientation)

        # instantaneous angle error
        dyaw = np.arctan2(
            np.sin(yaw1 - yaw2),
            np.cos(yaw1 - yaw2)
        )

        self.yaw_err_buf.append(dyaw)
        if len(self.yaw_err_buf) > self.WINDOW:
            self.yaw_err_buf.pop(0)

        yaw_rmse = np.sqrt(np.mean(np.array(self.yaw_err_buf)**2))
        yaw_rmse_deg = np.degrees(yaw_rmse)


        return pos_rmse, yaw_rmse_deg

    def update_plot(self):
        if self.odom is None or self.gt is None:
            return
        
        stamp = self.odom.header.stamp
        t_msg = stamp.sec + stamp.nanosec * 1e-9

        if self.last_stamp == t_msg:
            return  # no new data

        self.last_stamp = t_msg

        if self.t0 is None:
            self.t0 = t_msg
            self.last_time = 0.0
            return
        
        pos_err, yaw_err = self.compute_error()

        t = t_msg - self.t0
        dt = t - self.last_time
        self.last_time = t

        self.time.append(t)
        self.pos_err.append(pos_err)
        self.yaw_err.append(yaw_err)


        


        if not self.stable:
            if pos_err < self.RMSE_THRESH:
                self.stable_duration += dt
                if self.stable_duration >= self.STABLE_TIME:
                    self.stable = True
                    self.t_stable = t
                    self.get_logger().info(
                        f"Odometry stabilized at t = {t:.2f} s (RMSE < {self.RMSE_THRESH})\n"
                        f"Current RMSE is {pos_err} m"
                    )
            else:
                self.stable_duration = 0.0

        if self.stable and not hasattr(self, '_annotated'):
            self.ax_pos.axvline(
                self.t_stable,
                color='green',
                linestyle='--',
                label='Stable'
            )
            self.ax_pos.legend()
            self._annotated = True

        if not self.yaw_stable:
            if yaw_err < self.YAW_RMSE_THRESH:
                self.yaw_stable_duration += dt
                if self.yaw_stable_duration >= self.STABLE_TIME:
                    self.yaw_stable = True
                    self.t_yaw_stable = t
                    self.get_logger().info(
                        f"Yaw stabilized at t = {t:.2f} s "
                        f"(RMSE < {self.YAW_RMSE_THRESH:.1f} deg)"
                        f"yaw rmse: {yaw_err}"
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



        if self.stable:
            self.tracking_rmse_buf.append(pos_err)

        if self.stable and len(self.tracking_rmse_buf) > 0:
            tracking_rmse = np.mean(self.tracking_rmse_buf)

            if self.tracking_line is None:
                self.tracking_line = self.ax_pos.axhline(
                    tracking_rmse,
                    linestyle=':',
                    color='red',
                    label='Tracking RMSE (mean)'
                )
                self.ax_pos.legend()
            else:
                self.tracking_line.set_ydata([tracking_rmse, tracking_rmse])

        if self.stable and len(self.tracking_rmse_buf) % 50 == 0:
            self.get_logger().info(
                f"Current tracking RMSE (mean) = {tracking_rmse:.3f} m"
            )



        # Position plot update
        self.l_pos.set_data(self.time, self.pos_err)
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()
        self.fig_pos.canvas.draw()
        self.fig_pos.canvas.flush_events()

        # Yaw plot update
        self.l_yaw.set_data(self.time, self.yaw_err)
        self.ax_yaw.relim()
        self.ax_yaw.autoscale_view()
        self.fig_yaw.canvas.draw()
        self.fig_yaw.canvas.flush_events()

        if t >= self.T_END:
            if self.stable and len(self.tracking_rmse_buf) > 0:
                final_tracking_rmse = np.mean(self.tracking_rmse_buf)
                self.get_logger().info(
                    f"Final tracking RMSE = {final_tracking_rmse:.3f} m"
                )

            self.get_logger().info("Simulation finished.")
            rclpy.shutdown()
        


        




def main():
    rclpy.init()
    node = OdomConvergencePlot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

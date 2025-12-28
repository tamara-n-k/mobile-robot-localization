import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


class OdomConvergencePlot(Node):

    def __init__(self):
        super().__init__('odom_convergence_plot')

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

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update_plot)

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

        # Yaw RMSE plot
        self.fig_yaw, self.ax_yaw = plt.subplots()
        self.fig_yaw.suptitle("Odometry Yaw RMSE")
        self.l_yaw, = self.ax_yaw.plot([], [], label="Yaw RMSE [deg]")
        self.ax_yaw.set_xlabel("Time [s]")
        self.ax_yaw.set_ylabel("RMSE [deg]")
        self.ax_yaw.legend()
        self.ax_yaw.grid(True)


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

        pos_err, yaw_err = self.compute_error()

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        self.pos_err.append(pos_err)
        self.yaw_err.append(yaw_err)
        self.time.append(t)

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



def main():
    rclpy.init()
    node = OdomConvergencePlot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

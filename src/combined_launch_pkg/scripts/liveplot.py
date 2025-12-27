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
        self.yaw_err = []
        self.time = []

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update_plot)

        # matplotlib
        plt.ion() # enables interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)

        self.l_pos, = self.ax1.plot([], [], label='Position error [m]')
        self.l_yaw, = self.ax2.plot([], [], label='Yaw error [deg]')

        # threshold lines
        self.ax1.axhline(0.05, color='r', linestyle='--', label='5 cm')
        self.ax2.axhline(2.0, color='r', linestyle='--', label='2 deg')

        self.ax1.legend()
        self.ax2.legend()

        self.ax1.set_ylabel("Position error [m]")
        self.ax2.set_ylabel("Yaw error [deg]")
        self.ax2.set_xlabel("Time [s]")

    def odom_cb(self, msg):
        self.odom = msg

    def gt_cb(self, msg):
        self.gt = msg

    def yaw_from_quat(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('xyz')[2]

    def compute_error(self):
        p1 = self.odom.pose.pose.position
        p2 = self.gt.pose.pose.position

        pos_err = np.linalg.norm([
            p1.x - p2.x,
            p1.y - p2.y,
            p1.z - p2.z
        ])

        q1 = self.odom.pose.pose.orientation
        q2 = self.gt.pose.pose.orientation

        yaw1 = self.yaw_from_quat(self.odom.pose.pose.orientation)
        yaw2 = self.yaw_from_quat(self.gt.pose.pose.orientation)


        yaw_err = np.degrees(np.arctan2(
            np.sin(yaw1 - yaw2),
            np.cos(yaw1 - yaw2)
        ))

        return pos_err, abs(yaw_err)

    def update_plot(self):
        if self.odom is None or self.gt is None:
            return

        pos_err, yaw_err = self.compute_error()

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        self.pos_err.append(pos_err)
        self.yaw_err.append(yaw_err)
        self.time.append(t)

        self.l_pos.set_data(self.time, self.pos_err)
        self.l_yaw.set_data(self.time, self.yaw_err)

        self.ax1.relim()
        self.ax1.autoscale_view()

        self.ax2.relim()
        self.ax2.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = OdomConvergencePlot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

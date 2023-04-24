import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Pose
import numpy as np
import filterpy.kalman
import filterpy.common


class Kalman(Node):
    def __init__(self):
        super().__init__('kalman_3d')
        self.accel_sub = self.create_subscription(
            Accel, 'pix_accel',
            self.accel_cb, 10)
        self.pose_sub = self.create_subscription(
            Pose, 'gnss_pose',
            self.pose_cb, 10)
        self.filter_pub = self.create_publisher(Pose, 'filter_pose')
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.sigma_acc = 0.01
        self.sigma_xy = 0.5
        self.sigma_z = 0.3
        self.sigma_model = 1e-4
        self.dt = 0.02

        # KF for X coord
        self.filter_x = filterpy.kalman.KalmanFilter(dim_x=3, dim_z=2)
        self.filter_x.F = np.array([[1, self.dt, (self.dt ** 2) / 2],
                                    [0, 1.0, self.dt],
                                    [0, 0, 1.0]])
        self.filter_x.H = np.array([[0.0, 0.0, 1.0],
                                    [1.0, 0.0, 0.0]])
        self.filter_x.Q = filterpy.common.Q_discrete_white_noise(dim=3, dt=self.dt,
                                                                 var=self.sigma_model)
        self.filter_x.R = np.array([[self.sigma_acc ** 2, 0],
                                    [0, self.sigma_xy ** 2]])
        self.filter_x.x = np.array([0.0, 0.0, 0.0])
        self.filter_x.P = np.array([[2.0, 0.0, 0.0],
                                    [0.0, 2.0, 0.0],
                                    [0.0, 0.0, 2.0]])
        # KF for Y coord
        self.filter_y = filterpy.kalman.KalmanFilter(dim_x=3, dim_z=2)
        self.filter_y.F = np.array([[1, self.dt, (self.dt ** 2) / 2],
                                    [0, 1.0, self.dt],
                                    [0, 0, 1.0]])
        self.filter_y.H = np.array([[0.0, 0.0, 1.0],
                                    [1.0, 0.0, 0.0]])
        self.filter_y.Q = filterpy.common.Q_discrete_white_noise(dim=3, dt=self.dt,
                                                                 var=self.sigma_model)
        self.filter_y.R = np.array([[self.sigma_acc ** 2, 0],
                                    [0, self.sigma_xy ** 2]])
        self.filter_y.x = np.array([0.0, 0.0, 0.0])
        self.filter_y.P = np.array([[2.0, 0.0, 0.0],
                                    [0.0, 2.0, 0.0],
                                    [0.0, 0.0, 2.0]])
        # KF for Z coord
        self.filter_z = filterpy.kalman.KalmanFilter(dim_x=3, dim_z=2)
        self.filter_z.F = np.array([[1, self.dt, (self.dt ** 2) / 2],
                                    [0, 1.0, self.dt],
                                    [0, 0, 1.0]])
        self.filter_z.H = np.array([[0.0, 0.0, 1.0],
                                    [1.0, 0.0, 0.0]])
        self.filter_z.Q = filterpy.common.Q_discrete_white_noise(dim=3, dt=self.dt,
                                                                 var=self.sigma_model)
        self.filter_z.R = np.array([[self.sigma_acc ** 2, 0],
                                    [0, self.sigma_z ** 2]])
        self.filter_z.x = np.array([0.0, 0.0, 0.0])
        self.filter_z.P = np.array([[2.0, 0.0, 0.0],
                                    [0.0, 2.0, 0.0],
                                    [0.0, 0.0, 2.0]])

    def accel_cb(self, msg):
        # ??
        self.ax = msg.linear.x
        self.ay = msg.linear.y
        self.az = msg.linear.z

        # measuring
        z_x = np.array([self.ax, self.x])
        z_y = np.array([self.ay, self.y])
        z_z = np.array([self.az, self.z])
        # prediction
        self.filter_x.predict()
        self.filter_y.predict()
        self.filter_z.predict()
        # correction
        self.filter_x.update(z_x)
        self.filter_y.update(z_y)
        self.filter_z.update(z_z)
        # linear cords
        filter_msg = Pose()
        filter_msg.position.x = self.filter_x.x[0]
        filter_msg.position.y = self.filter_y.x[0]
        filter_msg.position.z = self.filter_z.x[0]
        self.filter_pub.publish(filter_msg)

    def pose_cb(self, msg):
        # ENU
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z


def main(args=None):
    rclpy.init(args=args)
    kalman = Kalman()
    rclpy.spin(kalman)


if __name__ == '__main__':
    main()

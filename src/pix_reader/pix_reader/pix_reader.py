import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Accel
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as Rot


class MavParser:
    def __init__(self, port):
        self.connection = mavutil.mavlink_connection(port, baud=115200)
        self.imu_data = np.zeros(8)

    def parse_imu(self):
        att = self.connection.recv_match(type='ATTITUDE', blocking=True)
        imu = self.connection.recv_match(type='HIGHRES_IMU', blocking=True)
        self.imu_data = np.array([att.roll, att.pitch, att.yaw,
                                  imu.xacc, imu.yacc, imu.zacc,
                                  att.time_boot_ms, imu.time_usec])
        # print(att.roll, att.pitch, att.yaw)
        return self.imu_data


class PixPub(Node):
    def __init__(self):
        super().__init__('pix_reader')
        self.publisher_ = self.create_publisher(Accel, 'pix_accel', 10)
        timer_period = 0.01
        self.create_timer(timer_period, self.timer_cb)
        self.parser = MavParser("/dev/ttyACM1")
        self.g_acc = np.array([0.0, 0.0, 9.81])

    def get_glob_acc(self, imu_data):
        accel_global = np.zeros(4)
        accel_global[3] = imu_data[6]
        acc_rot = Rot.from_euler('xyz', imu_data[0:3])
        acc_with_g = acc_rot.apply(imu_data[3:6])
        accel_global[0:3] = acc_with_g + self.g_acc
        return accel_global

    def timer_cb(self):
        data = self.parser.parse_imu()
        acc_glob = self.get_glob_acc(data)
        msg = Accel()
        msg.linear.x = acc_glob[0]
        msg.linear.y = acc_glob[1]
        msg.linear.z = acc_glob[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pix_pub = PixPub()
    rclpy.spin(pix_pub)


if __name__ == '__main__':
    main()


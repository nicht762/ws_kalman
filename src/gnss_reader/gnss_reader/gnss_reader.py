import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
import pymap3d
from serial import Serial
from pynmeagps import NMEAReader, NMEAMessage


class GnssPub(Node):
    def __init__(self):
        super().__init__('gnss_reader')
        self.publisher_ = self.create_publisher(Pose, 'gnss_pose', 10)
        timer_period = 0.5
        self.create_timer(timer_period, self.timer_cb)

        self.ellipsoid = pymap3d.Ellipsoid()
        # base
        self.lat_0 = 55.760057829964516
        self.lon_0 = 48.75896086562219
        self.hgt_0 = 199.37

        self.stream = Serial('/dev/ttyACM0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)

    def timer_cb(self):
        for (raw_data, parsed_data) in self.nmr:
            if parsed_data.msgID == 'GGA':
                # print(parsed_data)
                east, north, up = pymap3d.geodetic2enu(parsed_data.lat, parsed_data.lon, parsed_data.alt,
                                                       self.lat_0, self.lon_0, self.hgt_0,
                                                       ell=self.ellipsoid, deg=True)
                # print(east, north, up)
                msg = Pose()
                # ENU
                msg.position.x = east
                msg.position.y = north
                msg.position.z = up
                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gnss_pub = GnssPub()
    rclpy.spin(gnss_pub)


if __name__ == '__main__':
    main()

import os
import utm
import time
import math
import yaml
import rclpy

import numpy as np
import terasim_cosim.redis_msgs as redis_msgs

from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from math import atan2, pi, cos, sin

from geometry_msgs.msg import PoseWithCovarianceStamped

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


def latlon_to_utm(lat, lon):
    """Convert latitude/longitude to UTM coordinates (Zone 17N for Michigan)."""
    # WGS84 parameters
    a = 6378137.0  # equatorial radius
    e = 0.0818191908426  # eccentricity
    k0 = 0.9996  # scale factor

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # UTM zone 17 central meridian is -81 degrees
    lon0 = math.radians(-81)

    n = a / math.sqrt(1 - e**2 * math.sin(lat_rad)**2)
    t = math.tan(lat_rad)**2
    c = (e**2 / (1 - e**2)) * math.cos(lat_rad)**2
    A = (lon_rad - lon0) * math.cos(lat_rad)

    m = a * (
        (1 - e**2/4 - 3*e**4/64 - 5*e**6/256) * lat_rad
        - (3*e**2/8 + 3*e**4/32 + 45*e**6/1024) * math.sin(2*lat_rad)
        + (15*e**4/256 + 45*e**6/1024) * math.sin(4*lat_rad)
        - (35*e**6/3072) * math.sin(6*lat_rad)
    )

    easting = k0 * n * (A + (1-t+c)*A**3/6 + (5-18*t+t**2+72*c-58*(e**2/(1-e**2)))*A**5/120) + 500000
    northing = k0 * (m + n * math.tan(lat_rad) * (A**2/2 + (5-t+9*c+4*c**2)*A**4/24 + (61-58*t+t**2+600*c-330*(e**2/(1-e**2)))*A**6/720))

    return easting, northing


class GnssDecoder(Node):

    def __init__(self):
        super().__init__("gnss_decoder")

        self.sub_imu = self.create_subscription(Imu, "/ins/imu", self.imu_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/ins/odometry", self.odom_callback, 10
        )
        self.sub_nav_sat_fix = self.create_subscription(
            NavSatFix, "/ins/nav_sat_fix", self.gnss_callback, 10
        )

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/mcity/cav_pose", 10
        )

        self.timer = self.create_timer(0.02, self.on_timer)

        self.saved_odom_msg = None
        self.saved_imu_msg = None
        self.saved_nav_msg = None

        # Configure redis key-and data type
        key_value_config = {CAV_INFO: redis_msgs.ActorDict}
        self.redis_client = create_redis_client(key_value_config=key_value_config)

        # Map path for reading map_projector_info.yaml
        self.declare_parameter("map_path", os.path.expanduser("~/autoware/map"))
        self.map_path = self.get_parameter("map_path").value

        # Load UTM offset dynamically from map_projector_info.yaml
        self.UTM_offset = self._load_utm_offset_from_map()

        print("Reading GNSS info and set to cosim...")

    def _load_utm_offset_from_map(self):
        """Load UTM offset from map_projector_info.yaml in the map directory."""
        yaml_path = os.path.join(self.map_path, "map_projector_info.yaml")

        # Default fallback: Mcity coordinates
        default_offset = [-277497.10, -4686518.71]

        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)

            if 'map_origin' in config:
                lat = config['map_origin']['latitude']
                lon = config['map_origin']['longitude']

                easting, northing = latlon_to_utm(lat, lon)
                utm_offset = [-easting, -northing]

                self.get_logger().info(
                    f"Loaded map origin from {yaml_path}: lat={lat}, lon={lon}"
                )
                self.get_logger().info(
                    f"Computed UTM offset: [{utm_offset[0]:.2f}, {utm_offset[1]:.2f}]"
                )
                return utm_offset
            else:
                self.get_logger().warn(
                    f"No 'map_origin' found in {yaml_path}, using default Mcity offset"
                )
        except FileNotFoundError:
            self.get_logger().warn(
                f"Map projector info not found at {yaml_path}, using default Mcity offset"
            )
        except Exception as e:
            self.get_logger().error(
                f"Error loading map projector info: {e}, using default Mcity offset"
            )

        return default_offset

    def on_timer(self):
        if self.saved_nav_msg and self.saved_odom_msg and self.saved_imu_msg:
            self.sync_gnss_cav_to_cosim()

    def sync_gnss_cav_to_cosim(self):
        if (
            self.saved_nav_msg is None
            or self.saved_odom_msg is None
            or self.saved_imu_msg is None
        ):
            return

        lat = self.saved_nav_msg.latitude
        lon = self.saved_nav_msg.longitude
        utm_coords = utm.from_latlon(lat, lon)

        x = utm_coords[0]
        y = utm_coords[1]
        z = self.saved_nav_msg.altitude

        orientation = self.get_vehicle_orientation()
        speed_long = self.get_vehicle_speed()

        # Modify position based on some offsets and set to pose message
        pose_with_cov_msg = PoseWithCovarianceStamped()
        pose_with_cov_msg.pose.pose.position.x = x + self.UTM_offset[0]
        pose_with_cov_msg.pose.pose.position.y = y + self.UTM_offset[1]

        # Extract and process orientation data
        qx, qy, qz, qw = self.get_quaternion_from_orientation(orientation)

        pose_with_cov_msg.pose.pose.orientation.x = qx
        pose_with_cov_msg.pose.pose.orientation.y = qy
        pose_with_cov_msg.pose.pose.orientation.z = qz
        pose_with_cov_msg.pose.pose.orientation.w = qw

        self.pub_pose.publish(pose_with_cov_msg)

        # For detailed fileds, see redis_msgs/VehicleDict.py
        cav_info = redis_msgs.ActorDict()

        # Set the timestamp
        cav_info.header.timestamp = time.time()

        # For detailed fileds, see redis_msgs/Vehicle.py
        cav = redis_msgs.Actor()
        cav.x = x
        cav.y = y
        cav.z = z
        cav.length = 5.0
        cav.width = 1.8
        cav.height = 1.5
        cav.orientation = orientation
        cav.speed_long = speed_long

        # Add bv to terasim_cosim_vehicle_info. You can add as many vehicles as you want.
        cav_info.data["CAV"] = cav

        self.redis_client.set(CAV_INFO, cav_info)

        self.saved_nav_msg = None
        self.saved_odom_msg = None
        self.saved_imu_msg = None

    def get_vehicle_speed(self):
        vx = self.saved_odom_msg.twist.twist.linear.x
        vy = self.saved_odom_msg.twist.twist.linear.y
        vz = self.saved_odom_msg.twist.twist.linear.z

        speed = np.sqrt(vx**2 + vy**2 + vz**2)

        # filter out noise
        if speed < 0.05:
            speed = 0.00

        return speed

    def get_vehicle_orientation(self):
        qx = self.saved_imu_msg.orientation.x
        qy = self.saved_imu_msg.orientation.y
        qz = self.saved_imu_msg.orientation.z
        qw = self.saved_imu_msg.orientation.w

        # create the quaternion
        quat = R.from_quat([qx, qy, qz, qw])

        # define the rotations
        rotationAroundX = R.from_euler("x", 90, degrees=True)
        rotationAroundY = R.from_euler("y", -90, degrees=True)
        rotationAroundZ = R.from_euler("z", 90, degrees=True)

        # apply the rotations
        quat = quat * rotationAroundX * rotationAroundY * rotationAroundZ

        # extract the updated quaternion components
        qx, qy, qz, qw = quat.as_quat()

        orientation = self.get_orientation_from_quaternion(qx, qy, qz, qw)

        return orientation

    def get_orientation_from_quaternion(self, qx, qy, qz, qw):
        orientation = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        while orientation > pi:
            orientation -= 2.0 * pi
        while orientation < -pi:
            orientation += 2.0 * pi

        return orientation

    def get_quaternion_from_orientation(self, orientation):
        w = cos(orientation / 2.0)
        x = 0.0
        y = 0.0
        z = sin(orientation / 2.0)

        return x, y, z, w

    def imu_callback(self, msg):
        self.saved_imu_msg = msg

    def odom_callback(self, msg):
        self.saved_odom_msg = msg

    def gnss_callback(self, msg):
        self.saved_nav_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = GnssDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

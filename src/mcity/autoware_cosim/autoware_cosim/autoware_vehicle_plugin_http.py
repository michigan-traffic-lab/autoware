import time
import math
import os
import yaml
import rclpy
import tf2_ros
import numpy as np
import requests
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from autoware_auto_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    ObjectClassification,
    Shape,
)

from math import atan2, cos, sin, pi
from pyproj import Proj


def latlon_to_utm(lat, lon):
    """Convert latitude/longitude to UTM coordinates."""
    # Determine UTM zone from longitude
    zone = int((lon + 180) / 6) + 1
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', datum='WGS84')
    x, y = proj(lon, lat)
    return x, y


class AutowareVehiclePluginHTTP(Node):

    def __init__(self):
        super().__init__("autoware_vehicle_plugin_http")

        self.declare_parameter("control_cav", True)
        self.declare_parameter("http_host", "localhost")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("map_path", os.path.expanduser("~/autoware/map"))

        self.control_cav = self.get_parameter("control_cav").value
        self.http_host = self.get_parameter("http_host").value
        self.http_port = self.get_parameter("http_port").value
        self.map_path = self.get_parameter("map_path").value

        self.base_url = f"http://{self.http_host}:{self.http_port}"

        # autoware cav localization and display
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            10,
        )
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            "/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        self.pub_odom = self.create_publisher(
            Odometry, "/localization/pose_twist_fusion_filter/kinematic_state", 10
        )

        # autoware perception
        self.pub_detected_objects = self.create_publisher(
            DetectedObjects, "/perception/object_recognition/detection/objects", 10
        )

        self.sub_ego_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        self.tf = tf2_ros.TransformBroadcaster(self)

        self.cav_timer = self.create_timer(0.02, self.on_cav_timer)
        self.bv_timer = self.create_timer(0.1, self.on_bv_timer)

        self.saved_odom_msg = Odometry()

        # Load UTM offset dynamically from map_projector_info.yaml
        self.UTM_offset = self._load_utm_offset()

        print(f"Starting autoware vehicle interface with HTTP at {self.base_url}...")
        print(f"UTM offset loaded: {self.UTM_offset}")

    def _load_utm_offset(self):
        """Load UTM offset from map_projector_info.yaml in the map directory."""
        default_offset = [0.0, 0.0, 0.0]

        map_projector_path = os.path.join(self.map_path, "map_projector_info.yaml")

        if not os.path.exists(map_projector_path):
            self.get_logger().warn(f"map_projector_info.yaml not found at {map_projector_path}, using default offset")
            return default_offset

        try:
            with open(map_projector_path, 'r') as f:
                config = yaml.safe_load(f)

            map_origin = config.get("map_origin", {})
            lat = map_origin.get("latitude")
            lon = map_origin.get("longitude")
            alt = map_origin.get("altitude", 0.0)

            if lat is None or lon is None:
                self.get_logger().warn("map_origin latitude/longitude not found, using default offset")
                return default_offset

            # Convert lat/lon to UTM and negate (to match SUMO netOffset convention)
            utm_x, utm_y = latlon_to_utm(lat, lon)
            utm_offset = [-utm_x, -utm_y, -alt]

            self.get_logger().info(f"Loaded map origin: lat={lat}, lon={lon}")
            self.get_logger().info(f"Computed UTM offset: {utm_offset}")

            return utm_offset

        except Exception as e:
            self.get_logger().error(f"Failed to load map_projector_info.yaml: {e}")
            return default_offset

    def on_cav_timer(self):
        if self.control_cav:
            self.sync_autoware_cav_to_http()
        else:
            self.sync_http_cav_to_autoware()

    def on_bv_timer(self):
        self.sync_http_vehicle_to_autoware()

    def sync_autoware_cav_to_http(self):
        x = self.saved_odom_msg.pose.pose.position.x - self.UTM_offset[0]
        y = self.saved_odom_msg.pose.pose.position.y - self.UTM_offset[1]
        speed_long = self.saved_odom_msg.twist.twist.linear.x

        qx = self.saved_odom_msg.pose.pose.orientation.x
        qy = self.saved_odom_msg.pose.pose.orientation.y
        qz = self.saved_odom_msg.pose.pose.orientation.z
        qw = self.saved_odom_msg.pose.pose.orientation.w

        orientation = self.get_orientation_from_quaternion(qx, qy, qz, qw)
        x, y = self.autoware_coordinate_to_center_coordinate(x, y, orientation)

        cav_data = {
            "x": x,
            "y": y,
            "z": 275.0,
            "orientation": orientation,
            "speed_long": speed_long,
            "length": 5.0,
            "width": 1.8,
            "height": 1.5,
        }

        try:
            requests.post(f"{self.base_url}/cav", json=cav_data, timeout=0.1)
        except requests.exceptions.RequestException as e:
            pass  # Silently ignore connection errors

    def sync_http_cav_to_autoware(self):
        try:
            response = requests.get(f"{self.base_url}/cav", timeout=0.1)
            cav_info = response.json()
        except requests.exceptions.RequestException:
            return

        if not cav_info:
            return

        cav_x = cav_info["x"] + self.UTM_offset[0]
        cav_y = cav_info["y"] + self.UTM_offset[1]
        cav_z = 0.0

        cav_orientation = cav_info["orientation"]

        cav_x, cav_y = self.center_coordinate_to_autoware_coordinate(
            cav_x, cav_y, cav_orientation
        )

        # Modify position based on some offsets and set to pose message
        pose_with_cov_msg = PoseWithCovarianceStamped()
        pose_with_cov_msg.pose.pose.position.x = cav_x
        pose_with_cov_msg.pose.pose.position.y = cav_y

        # Extract and process orientation data
        qx, qy, qz, qw = self.get_quaternion_from_orientation(cav_orientation)

        pose_with_cov_msg.pose.pose.orientation.x = qx
        pose_with_cov_msg.pose.pose.orientation.y = qy
        pose_with_cov_msg.pose.pose.orientation.z = qz
        pose_with_cov_msg.pose.pose.orientation.w = qw

        # Set linear velocities from odom to twist message
        twist_with_cov_msg = TwistWithCovarianceStamped()

        twist_with_cov_msg.twist.twist.linear.x = cav_info["speed_long"]
        twist_with_cov_msg.twist.twist.linear.y = 0.0
        twist_with_cov_msg.twist.twist.linear.z = 0.0

        pose_with_cov_msg.pose.covariance = np.eye(6).flatten().tolist()
        twist_with_cov_msg.twist.covariance = np.eye(6).flatten().tolist()

        odom_msg = Odometry()
        odom_msg.pose.pose = pose_with_cov_msg.pose.pose
        odom_msg.twist.twist = twist_with_cov_msg.twist.twist

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Assign the same header to all messages
        pose_with_cov_msg.header = header
        twist_with_cov_msg.header = header
        odom_msg.header = header

        # Publish messages
        self.pub_pose.publish(pose_with_cov_msg)
        self.pub_twist.publish(twist_with_cov_msg)
        self.pub_odom.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = cav_x
        t.transform.translation.y = cav_y
        t.transform.translation.z = cav_z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf.sendTransform(t)

    def sync_http_vehicle_to_autoware(self):
        try:
            response = requests.get(f"{self.base_url}/actors", timeout=0.1)
            actors_data = response.json()
        except requests.exceptions.RequestException:
            return

        if not actors_data or "actors" not in actors_data:
            return

        self.detected_objects_msg = DetectedObjects()

        for vehID, veh_info in actors_data["actors"].items():
            if vehID != "CAV":
                self.update_perception_in_autoware(vehID, veh_info)

        self.detected_objects_msg.header.stamp = self.get_clock().now().to_msg()
        self.detected_objects_msg.header.frame_id = "map"
        self.pub_detected_objects.publish(self.detected_objects_msg)

    def get_pose_with_variance(self, bv_info):
        bv_pose_with_covariance = PoseWithCovariance()

        x = bv_info["x"] + self.UTM_offset[0]
        y = bv_info["y"] + self.UTM_offset[1]
        orientation = bv_info["orientation"]

        bv_pose_with_covariance.pose.position.x = x
        bv_pose_with_covariance.pose.position.y = y
        bv_pose_with_covariance.pose.position.z = 0.8
        bv_pose_with_covariance.pose.orientation.w = cos(orientation / 2)
        bv_pose_with_covariance.pose.orientation.x = 0.0
        bv_pose_with_covariance.pose.orientation.y = 0.0
        bv_pose_with_covariance.pose.orientation.z = sin(orientation / 2)

        return bv_pose_with_covariance

    def get_twist_with_variance(self, bv_info):
        bv_twist_with_covariance = TwistWithCovariance()
        bv_twist_with_covariance.twist.linear.x = bv_info["speed_long"]
        return bv_twist_with_covariance

    def get_shape(self, bv_info):
        bv_shape = Shape()
        bv_shape.type = Shape.BOUNDING_BOX
        bv_shape.dimensions.x = bv_info["length"]
        bv_shape.dimensions.y = bv_info["width"]
        bv_shape.dimensions.z = bv_info["height"]
        return bv_shape

    def get_classification(self, bv_key, bv_info=None):
        bv_classification = ObjectClassification()

        # Check vehicle type from bv_info first
        veh_type = bv_info.get("type", "").lower() if bv_info else ""
        bv_key_lower = bv_key.lower()

        if "POV" in bv_key or "BV" in bv_key or "CV" in bv_key or "CARLA" in bv_key:
            bv_classification.label = ObjectClassification.CAR
        elif "VRU" in bv_key or "pedestrian" in veh_type or "ped" in bv_key_lower:
            bv_classification.label = ObjectClassification.PEDESTRIAN
        elif "bike" in veh_type or "bicycle" in veh_type or "cyclist" in bv_key_lower:
            bv_classification.label = ObjectClassification.BICYCLE
        elif "motorcycle" in veh_type or "motorbike" in veh_type:
            bv_classification.label = ObjectClassification.MOTORCYCLE
        elif "truck" in veh_type:
            bv_classification.label = ObjectClassification.TRUCK
        elif "bus" in veh_type:
            bv_classification.label = ObjectClassification.BUS
        elif "trailer" in veh_type:
            bv_classification.label = ObjectClassification.TRAILER
        else:
            # Default to CAR for any vehicle (v_* IDs are vehicles)
            bv_classification.label = ObjectClassification.CAR

        bv_classification.probability = 1.0
        return bv_classification

    def update_perception_in_autoware(self, bv_key, bv_info):
        detected_object = DetectedObject()
        detected_object.existence_probability = 1.0
        detected_object.classification.append(self.get_classification(bv_key, bv_info))
        detected_object.kinematics.pose_with_covariance = self.get_pose_with_variance(
            bv_info
        )
        detected_object.kinematics.has_position_covariance = False
        detected_object.kinematics.orientation_availability = 0
        detected_object.kinematics.twist_with_covariance = self.get_twist_with_variance(
            bv_info
        )
        detected_object.kinematics.has_twist = False
        detected_object.kinematics.has_twist_covariance = False
        detected_object.shape = self.get_shape(bv_info)

        self.detected_objects_msg.objects.append(detected_object)

    def center_coordinate_to_autoware_coordinate(
        self, x, y, heading, rear_shaft_to_center=1.5
    ):
        x = x - math.cos(heading) * rear_shaft_to_center
        y = y - math.sin(heading) * rear_shaft_to_center
        return x, y

    def autoware_coordinate_to_center_coordinate(
        self, x, y, heading, rear_shaft_to_center=1.5
    ):
        x = x + math.cos(heading) * rear_shaft_to_center
        y = y + math.sin(heading) * rear_shaft_to_center
        return x, y

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

    def odom_callback(self, msg):
        self.saved_odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = AutowareVehiclePluginHTTP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


main()

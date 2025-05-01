import math
import json
import rclpy
import terasim_cosim.redis_msgs as redis_msgs

from rclpy.node import Node
from mcity_msgs.msg import VehiclePlanning
from autoware_auto_planning_msgs.msg import Trajectory


class AutowarePlanning(Node):
    def __init__(self):
        super().__init__("autoware_planning")

        # Register publisher
        self.pub_path = self.create_publisher(
            VehiclePlanning, "/mcity/vehicle_planning", 10
        )

        # Register subscriber
        self.sub_trajectory = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self.trajectory_callback,
            10,
        )

        # Register timer
        self.traj_timer = self.create_timer(0.05, self.on_timer)

        self.path_msg = VehiclePlanning()
        self.path_msg.estop = 0
        self.path_msg.go = 1

        print("reading trajectory from autoware and publishing preview control path...")

    def on_timer(self):
        self.path_msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self.pub_path.publish(self.path_msg)

    def trajectory_callback(self, msg):
        x_list = []
        y_list = []
        vd_list = []
        ori_list = []

        # 500 * 0.1 meter = 50m max distance
        max_num_points = 500
        num_points_to_store = min(max_num_points, len(msg.points))

        for i in range(num_points_to_store):
            point = msg.points[i]

            qx = point.pose.orientation.x
            qy = point.pose.orientation.y
            qz = point.pose.orientation.z
            qw = point.pose.orientation.w

            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            x_list.append(point.pose.position.x)
            y_list.append(point.pose.position.y)
            vd_list.append(point.longitudinal_velocity_mps)
            ori_list.append(yaw)

        self.path_msg.x_vector = x_list
        self.path_msg.y_vector = y_list
        self.path_msg.vd_vector = vd_list
        self.path_msg.ori_vector = ori_list


def main(args=None):
    rclpy.init(args=args)
    node = AutowarePlanning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

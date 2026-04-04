#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):

    def __init__(self):
        super().__init__('odom_to_path')

        # ✅ Get robot name from launch file
        self.declare_parameter('robot_name', 'robot_0')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        topic_odom = f'/{robot_name}/odom'
        topic_path = f'/{robot_name}/path'

        self.get_logger().info(f"[{robot_name}] Subscribing to {topic_odom}")
        self.get_logger().info(f"[{robot_name}] Publishing to {topic_path}")

        self.subscription = self.create_subscription(
            Odometry,
            topic_odom,
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Path,
            topic_path,
            10
        )

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.max_poses = 1000

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)

        # limit size
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
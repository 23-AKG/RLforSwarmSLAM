#!/usr/bin/env python3
"""
pointcloud_relay.py

Relays a PointCloud2 message from one topic to another,
rewriting the frame_id in the header.

This is needed because Swarm-SLAM expects the pointcloud
frame_id to be 'velodyne', but our sim publishes it as
'robot_N/lidar_link'.

Usage (via launch file):
  ros2 run swarm_slam_sim pointcloud_relay.py --ros-args
    -p input_topic:=/robot_0/lidar/points
    -p output_topic:=/r0/pointcloud
    -p output_frame:=velodyne
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointcloudRelay(Node):

    def __init__(self):
        super().__init__('pointcloud_relay')

        self.declare_parameter('input_topic',  '/robot_0/lidar/points')
        self.declare_parameter('output_topic', '/r0/pointcloud')
        self.declare_parameter('output_frame', 'velodyne')

        in_topic     = self.get_parameter('input_topic').value
        out_topic    = self.get_parameter('output_topic').value
        self.out_frame = self.get_parameter('output_frame').value

        self.get_logger().info(
            f'Relaying {in_topic} → {out_topic} '
            f'(frame_id rewritten to "{self.out_frame}")')

        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.callback,
            rclpy.qos.qos_profile_sensor_data)

    def callback(self, msg: PointCloud2):
        msg.header.frame_id = self.out_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

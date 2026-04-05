#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# 🔥 IMPORTANT: change if your type is different
from cslam_common_interfaces.msg import VizPointCloud


class UnifiedMapper(Node):

    def __init__(self):
        super().__init__('unified_mapper')

        # Map params
        self.resolution = 0.2
        self.size = 200 # 400
        self.origin = -20.0 # -40.0

        self.grid = np.zeros((self.size, self.size), dtype=np.int8)

        # Subscribe to CSLAM keyframe cloud
        self.create_subscription(
            VizPointCloud,
            '/cslam/viz/keyframe_pointcloud',
            self.cslam_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(OccupancyGrid, '/map_cslam', 10)

        self.get_logger().info("Unified Mapper Started")

    def world_to_grid(self, x, y):
        gx = int((x - self.origin) / self.resolution)
        gy = int((y - self.origin) / self.resolution)
        return gx, gy

    def cslam_callback(self, msg):

        # 🔥 FIX 1: correct field
        cloud = msg.pointcloud

        for p in pc2.read_points(cloud, field_names=('x','y','z'), skip_nans=True):
            x, y, z = p

            # height filter
            if z < -0.2 or z > 2.0:
                continue

            gx, gy = self.world_to_grid(x, y)

            if 0 <= gx < self.size and 0 <= gy < self.size:
                # self.grid[gx, gy] = 100
                self.grid[gx, gy] = min(100, self.grid[gx, gy] + 5)

        self.publish_map()

    def publish_map(self):

        grid_msg = OccupancyGrid()

        # 🔥 FIX 2: force frame
        grid_msg.header.frame_id = 'map'

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.size
        grid_msg.info.height = self.size

        grid_msg.info.origin.position.x = self.origin
        grid_msg.info.origin.position.y = self.origin

        grid_msg.data = self.grid.flatten().tolist()

        self.pub.publish(grid_msg)


def main():
    rclpy.init()
    node = UnifiedMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
odom_to_path.py
Converts odometry to nav_msgs/Path for RViz trail visualization.
One instance per robot.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('robot_name', 'robot_0')
        ns = self.get_parameter('robot_name').value
        self.path = Path()
        self.path.header.frame_id = f'{ns}/odom'
        self.pub = self.create_publisher(Path, f'/{ns}/path', 10)
        self.create_subscription(
            Odometry, f'/{ns}/odom', self.callback,
            rclpy.qos.qos_profile_sensor_data)

    def callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        # self.path.poses.append(pose)
        # only add if moved enough (denser + cleaner)
        if len(self.path.poses) == 0:
            self.path.poses.append(pose)
        else:
            last = self.path.poses[-1].pose.position
            dx = pose.pose.position.x - last.x
            dy = pose.pose.position.y - last.y
            if (dx*dx + dy*dy) > 0.05:   # threshold
                self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        if len(self.path.poses) > 3000:
            self.path.poses = self.path.poses[-3000:]
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

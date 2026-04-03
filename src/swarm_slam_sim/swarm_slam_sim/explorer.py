#!/usr/bin/env python3
"""
explorer.py — Autonomous exploration node for Swarm-SLAM sim

One instance per robot. Subscribes to LiDAR point cloud and publishes
cmd_vel to make the robot explore the environment autonomously.

Strategy: random walk with reactive LiDAR obstacle avoidance
  - Move forward at cruise speed
  - If obstacle detected ahead → stop and turn
  - Randomly vary heading every N seconds to avoid getting stuck
  - Emergency reverse if very close obstacle

Usage:
  ros2 run swarm_slam_sim explorer --ros-args -p robot_name:=robot_0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import random
import math


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('cruise_speed',    0.35)   # m/s forward
        self.declare_parameter('turn_speed',      0.6)    # rad/s
        self.declare_parameter('obstacle_dist',   1.2)    # m — start turning
        self.declare_parameter('danger_dist',     0.5)    # m — reverse
        self.declare_parameter('random_turn_interval', 8.0)  # s

        ns               = self.get_parameter('robot_name').value
        self.cruise      = self.get_parameter('cruise_speed').value
        self.turn_speed  = self.get_parameter('turn_speed').value
        self.obs_dist    = self.get_parameter('obstacle_dist').value
        self.danger_dist = self.get_parameter('danger_dist').value
        self.rnd_interval= self.get_parameter('random_turn_interval').value

        self.ns = ns
        self.get_logger().info(f'Explorer starting for {ns}')

        # ── State ─────────────────────────────────────────────────
        self.state          = 'forward'   # forward | turning | reverse
        self.turn_direction = 1.0         # +1 left, -1 right
        self.turn_duration  = 0.0
        self.turn_elapsed   = 0.0
        self.last_random_t  = self.get_clock().now()

        # Sector distances from last LiDAR scan
        # front, front-left, front-right, left, right
        self.sectors = {
            'front':       99.0,
            'front_left':  99.0,
            'front_right': 99.0,
        }

        # ── Publisher ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, f'/{ns}/cmd_vel', 10)

        # ── Subscriber — 3D LiDAR PointCloud2 ────────────────────
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            f'/{ns}/lidar/points',
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data)

        # ── Control loop at 10 Hz ─────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

    # ── LiDAR callback ────────────────────────────────────────────
    def lidar_callback(self, msg: PointCloud2):
        """
        Extract minimum distances in three forward sectors from
        the 3D point cloud. We project all points onto the XY plane
        (ignoring Z) and bucket by azimuth angle.

        Sectors (robot frame, X = forward):
          front       : azimuth in [-25°, +25°]
          front_left  : azimuth in [+25°, +70°]
          front_right : azimuth in [-70°, -25°]
        """
        front_min       = 99.0
        front_left_min  = 99.0
        front_right_min = 99.0

        try:
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True):
                x, y, z = p
                # ignore floor/ceiling returns
                if z < -0.05 or z > 2.5:
                    continue
                dist = math.sqrt(x*x + y*y)
                if dist < 0.15:   # too close to sensor origin — noise
                    continue
                angle = math.atan2(y, x)  # radians, 0 = straight ahead

                if   -0.44 <= angle <= 0.44:   # ±25°
                    front_min = min(front_min, dist)
                elif  0.44 <  angle <= 1.22:   # +25° to +70°
                    front_left_min = min(front_left_min, dist)
                elif -1.22 <= angle < -0.44:   # -70° to -25°
                    front_right_min = min(front_right_min, dist)
        except Exception as e:
            self.get_logger().warn(f'LiDAR parse error: {e}')
            return

        self.sectors['front']       = front_min
        self.sectors['front_left']  = front_left_min
        self.sectors['front_right'] = front_right_min

    # ── Control loop ──────────────────────────────────────────────
    def control_loop(self):
        dt = 0.1  # matches timer period

        front       = self.sectors['front']
        front_left  = self.sectors['front_left']
        front_right = self.sectors['front_right']

        cmd = Twist()

        # ── EMERGENCY REVERSE ─────────────────────────────────────
        if front <= self.danger_dist:
            if self.state != 'reverse':
                self.get_logger().info(
                    f'{self.ns}: DANGER — reversing (front={front:.2f}m)')
                self.state = 'reverse'
                # turn away from the closer side
                self.turn_direction = (
                    1.0 if front_right < front_left else -1.0)

            cmd.linear.x  = -0.15
            cmd.angular.z =  self.turn_direction * self.turn_speed
            self.cmd_pub.publish(cmd)
            return

        # ── OBSTACLE AHEAD → TURN ─────────────────────────────────
        if front <= self.obs_dist:
            if self.state != 'turning':
                # pick turn direction: away from the nearer side
                if front_right < front_left:
                    self.turn_direction = 1.0   # turn left
                else:
                    self.turn_direction = -1.0  # turn right

                # turn for a random duration [1.0, 2.5] s
                self.turn_duration = random.uniform(1.0, 2.5)
                self.turn_elapsed  = 0.0
                self.state = 'turning'
                self.get_logger().info(
                    f'{self.ns}: obstacle at {front:.2f}m — turning '
                    f'{"left" if self.turn_direction>0 else "right"} '
                    f'for {self.turn_duration:.1f}s')

            self.turn_elapsed += dt
            if self.turn_elapsed >= self.turn_duration:
                self.state = 'forward'
            else:
                cmd.linear.x  = 0.05   # creep forward while turning
                cmd.angular.z = self.turn_direction * self.turn_speed
                self.cmd_pub.publish(cmd)
                return

        # ── RANDOM HEADING CHANGE (prevent long straight corridors) ─
        now = self.get_clock().now()
        elapsed = (now - self.last_random_t).nanoseconds * 1e-9
        if elapsed > self.rnd_interval and self.state == 'forward':
            self.last_random_t = now
            self.turn_direction = random.choice([-1.0, 1.0])
            self.turn_duration  = random.uniform(0.5, 1.5)
            self.turn_elapsed   = 0.0
            self.state = 'turning'
            self.get_logger().info(
                f'{self.ns}: random turn '
                f'{"left" if self.turn_direction>0 else "right"}')
            return

        # ── FORWARD ───────────────────────────────────────────────
        self.state = 'forward'
        # slight steering correction if one side is tighter
        steer = 0.0
        if front_left < self.obs_dist * 1.5:
            steer = -0.2   # drift right
        elif front_right < self.obs_dist * 1.5:
            steer =  0.2   # drift left

        cmd.linear.x  = self.cruise
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

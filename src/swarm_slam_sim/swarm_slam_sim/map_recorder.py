#!/usr/bin/env python3
"""
map_recorder.py

Records ground truth trajectories and CSLAM optimized estimates
to JSON files for later comparison visualization.

Saves to ~/ROS2_SWARM/map_recordings/<timestamp>/
  ground_truth.json     — odom trajectories per robot
  cslam_estimates.json  — optimized keyframe poses per robot
  cslam_pointclouds.json — keyframe point clouds (x,y projected)

Usage:
  ros2 run swarm_slam_sim map_recorder.py

Stop with Ctrl+C — data is saved automatically on shutdown.
"""

import os
import json
import time
import math
import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

# These are custom Swarm-SLAM message types
try:
    from cslam_common_interfaces.msg import OptimizationResult, VizPointCloud
    CSLAM_MSGS = True
except ImportError:
    CSLAM_MSGS = False


class MapRecorder(Node):

    def __init__(self):
        super().__init__('map_recorder')

        self.declare_parameter('output_dir',
            os.path.expanduser('~/ROS2_SWARM/map_recordings'))
        self.declare_parameter('experiment_name', 'baseline')
        self.declare_parameter('robot_ids', [0, 1, 2])

        out_base  = self.get_parameter('output_dir').value
        exp_name  = self.get_parameter('experiment_name').value
        robot_ids = self.get_parameter('robot_ids').value

        # Create output directory
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(out_base, f'{exp_name}_{timestamp}')
        os.makedirs(self.out_dir, exist_ok=True)

        self.get_logger().info(
            f'MapRecorder: saving to {self.out_dir}')

        # Data stores
        self.ground_truth   = {f'robot_{i}': [] for i in robot_ids}
        self.cslam_poses    = {f'robot_{i}': {} for i in robot_ids}
        self.cslam_clouds   = []  # list of {robot_id, keyframe_id, points}

        # ── Ground truth subscribers (odom) ──────────────────────────────
        for i in robot_ids:
            self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',
                lambda msg, rid=i: self._odom_cb(msg, rid),
                qos_profile_sensor_data)

        # ── CSLAM subscribers ─────────────────────────────────────────────
        if CSLAM_MSGS:
            for i in robot_ids:
                self.create_subscription(
                    OptimizationResult,
                    f'/r{i}/cslam/optimized_estimates',
                    lambda msg, rid=i: self._opt_cb(msg, rid),
                    10)

            self.create_subscription(
                VizPointCloud,
                '/cslam/viz/keyframe_pointcloud',
                self._cloud_cb,
                10)
        else:
            self.get_logger().warn(
                'cslam_common_interfaces not found — '
                'only ground truth will be recorded')

        # Save every 30 seconds as backup
        self.create_timer(30.0, self._autosave)

        self.get_logger().info('Recording started. Ctrl+C to stop and save.')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg, robot_id):
        """Record odometry position every ~0.5m to avoid huge files."""
        key   = f'robot_{robot_id}'
        poses = self.ground_truth[key]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Only record if moved > 0.3m from last point
        if poses:
            last = poses[-1]
            if math.sqrt((x-last['x'])**2 + (y-last['y'])**2) < 0.3:
                return

        poses.append({'x': x, 'y': y, 't': t})

    def _opt_cb(self, msg, robot_id):
        """Record optimized keyframe poses."""
        key = f'robot_{robot_id}'
        for est in msg.estimates:
            kf_id = est.key.keyframe_id
            x = est.pose.position.x
            y = est.pose.position.y
            # Store latest estimate per keyframe (optimizer refines over time)
            self.cslam_poses[key][str(kf_id)] = {'x': x, 'y': y}

    def _cloud_cb(self, msg):
        """Record keyframe point clouds (project to x,y)."""
        robot_id    = msg.robot_id
        keyframe_id = msg.keyframe_id
        pc          = msg.pointcloud

        # Parse raw binary PointCloud2 data into (x, y) pairs
        points = []
        try:
            data = bytes(pc.data)
            step = pc.point_step
            for i in range(0, len(data) - step + 1, step):
                x = struct.unpack_from('<f', data, i)[0]
                y = struct.unpack_from('<f', data, i + 4)[0]
                if math.isfinite(x) and math.isfinite(y):
                    # Downsample: store every 3rd point to save space
                    if (i // step) % 3 == 0:
                        points.append([round(x, 3), round(y, 3)])
        except Exception as e:
            self.get_logger().warn(f'Cloud parse error: {e}')
            return

        # Only add if not already recorded for this keyframe
        existing = [c for c in self.cslam_clouds
                    if c['robot_id'] == robot_id
                    and c['keyframe_id'] == keyframe_id]
        if not existing and points:
            self.cslam_clouds.append({
                'robot_id':    robot_id,
                'keyframe_id': keyframe_id,
                'points':      points,
            })

    # ── Save ──────────────────────────────────────────────────────────────

    def save(self):
        # Ground truth
        gt_path = os.path.join(self.out_dir, 'ground_truth.json')
        with open(gt_path, 'w') as f:
            json.dump(self.ground_truth, f)
        self.get_logger().info(
            f'Saved ground truth: '
            + ', '.join(f'{k}:{len(v)} pts'
                        for k, v in self.ground_truth.items()))

        # CSLAM poses
        poses_path = os.path.join(self.out_dir, 'cslam_estimates.json')
        with open(poses_path, 'w') as f:
            json.dump(self.cslam_poses, f)
        self.get_logger().info(
            f'Saved CSLAM estimates: '
            + ', '.join(f'{k}:{len(v)} kfs'
                        for k, v in self.cslam_poses.items()))

        # CSLAM point clouds
        clouds_path = os.path.join(self.out_dir, 'cslam_pointclouds.json')
        with open(clouds_path, 'w') as f:
            json.dump(self.cslam_clouds, f)
        self.get_logger().info(
            f'Saved {len(self.cslam_clouds)} keyframe clouds')

        # Write a README
        readme_path = os.path.join(self.out_dir, 'README.txt')
        with open(readme_path, 'w') as f:
            f.write(f'Experiment: {self.get_parameter("experiment_name").value}\n')
            f.write(f'Recorded at: {time.strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            f.write('Files:\n')
            f.write('  ground_truth.json     — robot odometry trajectories\n')
            f.write('  cslam_estimates.json  — Swarm-SLAM keyframe poses\n')
            f.write('  cslam_pointclouds.json — keyframe point clouds\n\n')
            f.write('Visualize with:\n')
            f.write(f'  python3 visualize_maps.py {self.out_dir}\n')

        self.get_logger().info(f'All data saved to {self.out_dir}')

    def _autosave(self):
        self.get_logger().info('Auto-saving...')
        self.save()


def main(args=None):
    rclpy.init(args=args)
    node = MapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — saving final data...')
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
swarm_slam_bridge.launch.py

Bridges our Gazebo simulation to the Swarm-SLAM pipeline.

Key fix: pointcloud messages have their frame_id rewritten from
'robot_N/lidar_link' to 'velodyne' before being passed to Swarm-SLAM.
This is required because Swarm-SLAM's message filter does a TF lookup
using the pointcloud's frame_id.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_sim   = get_package_share_directory('swarm_slam_sim')
    pkg_cslam = get_package_share_directory('cslam_experiments')

    config_path = os.path.join(pkg_sim, 'config') + '/'
    config_file = 'swarm_slam_sim.yaml'

    robots = [
        {'id': 0, 'sim_ns': 'robot_0', 'cslam_ns': '/r0'},
        {'id': 1, 'sim_ns': 'robot_1', 'cslam_ns': '/r1'},
        {'id': 2, 'sim_ns': 'robot_2', 'cslam_ns': '/r2'},
    ]

    actions = []

    # ── 1. Simulation + exploration ───────────────────────────────
    actions.append(LogInfo(msg='[bridge] Starting Gazebo simulation...'))
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'explore.launch.py')
            )
        )
    )

    for r in robots:
        rid      = r['id']
        sim_ns   = r['sim_ns']
        cslam_ns = r['cslam_ns']

        # ── Pointcloud relay — rewrites frame_id to 'velodyne' ────
        # This is the critical fix. Swarm-SLAM's message filter looks
        # up frame_id in TF. Our sim frame is 'robot_N/lidar_link' but
        # Swarm-SLAM expects 'velodyne'. Rewriting the frame_id here
        # means Swarm-SLAM can find the transform via our static TF.
        lidar_relay = Node(
            package='swarm_slam_sim',
            executable='pointcloud_relay.py',
            name=f'lidar_relay_r{rid}',
            parameters=[{
                'input_topic':  f'/{sim_ns}/lidar/points',
                'output_topic': f'{cslam_ns}/pointcloud',
                'output_frame': 'velodyne',
                'use_sim_time': True,
            }],
            output='screen',
        )

        # ── Odometry relay — plain topic_tools relay ──────────────
        odom_relay = Node(
            package='topic_tools',
            executable='relay',
            name=f'odom_relay_r{rid}',
            arguments=[
                f'/{sim_ns}/odom',
                f'{cslam_ns}/odom',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # ── IMU relay ─────────────────────────────────────────────
        imu_relay = Node(
            package='topic_tools',
            executable='relay',
            name=f'imu_relay_r{rid}',
            arguments=[
                f'/{sim_ns}/imu/data',
                f'{cslam_ns}/imu/data',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # ── Static TF: velodyne → robot_N/base_footprint ──────────
        # Swarm-SLAM needs to know where 'velodyne' is relative to
        # the robot base. Since our LiDAR is mounted at the top of
        # the robot, we publish an identity transform here.
        # (Our URDF already handles the actual offset via robot_state_publisher)
        tf_velodyne = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_velodyne_r{rid}',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                f'{sim_ns}/base_footprint',
                'velodyne',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # ── Relay nodes start at t=8s ─────────────────────────────
        delayed_relays = TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg=f'[bridge] Starting relay nodes for r{rid}...'),
                lidar_relay,
                odom_relay,
                imu_relay,
            ]
        )

        # ── Swarm-SLAM starts staggered after relays ──────────────
        cslam_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_cslam, 'launch', 'cslam', 'cslam_lidar.launch.py'
                )
            ),
            launch_arguments={
                'config_path':                 config_path,
                'config_file':                 config_file,
                'robot_id':                    str(rid),
                'namespace':                   cslam_ns,
                'max_nb_robots':               '3',
                'enable_simulated_rendezvous': 'false',
                'rendezvous_schedule_file':    '',
            }.items(),
        )

        swarm_start_time = 20.0 + rid * 5.0
        delayed_cslam = TimerAction(
            period=swarm_start_time,
            actions=[
                LogInfo(msg=f'[bridge] Starting Swarm-SLAM for r{rid}...'),
                cslam_node,
            ]
        )

        actions.extend([
            tf_velodyne,
            delayed_relays,
            delayed_cslam,
        ])

    return LaunchDescription(actions)

"""
explore.launch.py — Sim + autonomous exploration + RViz visualizer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_slam_sim')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'multi_robot.launch.py')
        )
    )

    def make_explorer(robot_name, cruise, random_interval):
        return Node(
            package='swarm_slam_sim',
            executable='explorer.py',
            name=f'explorer_{robot_name}',
            parameters=[{
                'robot_name':           robot_name,
                'cruise_speed':         cruise,
                'turn_speed':           0.9,
                'obstacle_dist':        1.4,
                'danger_dist':          0.50,
                'random_turn_interval': random_interval,
                'use_sim_time':         True,
            }],
            output='screen',
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'swarm.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        sim_launch,
        make_explorer('robot_0', cruise=0.55, random_interval=7.0),
        make_explorer('robot_1', cruise=0.60, random_interval=9.0),
        make_explorer('robot_2', cruise=0.52, random_interval=6.0),
        rviz_node,
    ])

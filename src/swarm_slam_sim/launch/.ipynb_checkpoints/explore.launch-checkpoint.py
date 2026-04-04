"""
explore.launch.py

Two modes selectable via 'exploration_mode' argument:
  random    — random walk with obstacle avoidance (for development)
  waypoint  — predefined paths (for reproducible experiments + RL baseline)

Usage:
  ros2 launch swarm_slam_sim explore.launch.py exploration_mode:=waypoint
  ros2 launch swarm_slam_sim explore.launch.py exploration_mode:=random
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression

def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_slam_sim')

    mode_arg = DeclareLaunchArgument(
        'exploration_mode',
        default_value='waypoint',
        description='random or waypoint'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'multi_robot.launch.py')
        )
    )

    # ── Random explorers ─────────────────────────────────────────────────
    def make_explorer(robot_name, cruise, random_interval):
        return Node(
            package='swarm_slam_sim',
            executable='explorer.py',
            name=f'explorer_{robot_name}',
            condition=UnlessCondition(
                PythonExpression([
                    "'", LaunchConfiguration('exploration_mode'), "' == 'waypoint'"
                ])
            ),
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

    # ── Waypoint navigators ───────────────────────────────────────────────
    def make_navigator(robot_name, cruise=0.45):
        return Node(
            package='swarm_slam_sim',
            executable='waypoint_navigator.py',
            name=f'navigator_{robot_name}',
            condition=IfCondition(
                PythonExpression([
                   "'", LaunchConfiguration('exploration_mode'), "' == 'waypoint'"
                ])
            ),
            parameters=[{
                'robot_name':       robot_name,
                'cruise_speed':     cruise,
                'repeat_path':      True,
                'use_sim_time':     True,
            }],
            output='screen',
        )

    # ── Path visualization ────────────────────────────────────────────────
    def make_path_node(robot_name):
        return Node(
            package='swarm_slam_sim',
            executable='odom_to_path.py',
            name=f'odom_to_path_{robot_name}',
            parameters=[{
                'robot_name': robot_name,
                'use_sim_time': True,
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
        mode_arg,
        sim_launch,
        # Random explorers (only in random mode)
        make_explorer('robot_0', cruise=0.55, random_interval=7.0),
        make_explorer('robot_1', cruise=0.60, random_interval=9.0),
        make_explorer('robot_2', cruise=0.52, random_interval=6.0),
        # Waypoint navigators (only in waypoint mode)
        # robot_0 slightly slower — redundant path should be obvious in metrics
        make_navigator('robot_0', cruise=0.40),
        make_navigator('robot_1', cruise=0.45),
        make_navigator('robot_2', cruise=0.45),
        # Path visualization (always on)
        make_path_node('robot_0'),
        make_path_node('robot_1'),
        make_path_node('robot_2'),
        rviz_node,
    ])

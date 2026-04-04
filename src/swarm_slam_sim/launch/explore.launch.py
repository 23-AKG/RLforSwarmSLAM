"""
explore.launch.py

Two modes via 'exploration_mode' argument:
  waypoint (default) — predefined collision-free paths for reproducible experiments
  random             — random walk with obstacle avoidance (for development)

Usage:
  ros2 launch swarm_slam_sim explore.launch.py
  ros2 launch swarm_slam_sim explore.launch.py exploration_mode:=random
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_slam_sim')

    mode_arg = DeclareLaunchArgument(
        'exploration_mode', default_value='waypoint',
        description='waypoint or random')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'multi_robot.launch.py')))

    use_waypoint = PythonExpression(
        ["'", LaunchConfiguration('exploration_mode'), "' == 'waypoint'"])
    use_random = PythonExpression(
        ["'", LaunchConfiguration('exploration_mode'), "' != 'waypoint'"])

    def make_explorer(robot_name, cruise, interval):
        return Node(
            package='swarm_slam_sim', executable='explorer.py',
            name=f'explorer_{robot_name}',
            condition=IfCondition(use_random),
            parameters=[{
                'robot_name': robot_name, 'cruise_speed': cruise,
                'turn_speed': 0.9, 'obstacle_dist': 1.4,
                'danger_dist': 0.50, 'random_turn_interval': interval,
                'use_sim_time': True}],
            output='screen')

    def make_navigator(robot_name, cruise=0.40):
        return Node(
            package='swarm_slam_sim', executable='waypoint_navigator.py',
            name=f'navigator_{robot_name}',
            condition=IfCondition(use_waypoint),
            parameters=[{
                'robot_name': robot_name, 'cruise_speed': cruise,
                'repeat_path': True, 'use_sim_time': True}],
            output='screen')

    def make_path(robot_name):
        return Node(
            package='swarm_slam_sim', executable='odom_to_path.py',
            name=f'odom_to_path_{robot_name}',
            parameters=[{'robot_name': robot_name, 'use_sim_time': True}],
            output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'swarm.rviz')],
        parameters=[{'use_sim_time': True}], output='screen')

    return LaunchDescription([
        mode_arg, sim_launch,
        make_explorer('robot_0', 0.55, 7.0),
        make_explorer('robot_1', 0.60, 9.0),
        make_explorer('robot_2', 0.52, 6.0),
        make_navigator('robot_0', cruise=0.35),
        make_navigator('robot_1', cruise=0.42),
        make_navigator('robot_2', cruise=0.42),
        make_path('robot_0'),
        make_path('robot_1'),
        make_path('robot_2'),
        rviz,
    ])

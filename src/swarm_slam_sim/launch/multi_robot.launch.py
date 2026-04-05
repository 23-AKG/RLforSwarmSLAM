"""
swarm_slam_sim — multi_robot.launch.py

Launches:
  1. Gazebo Classic with parking_lot.world
  2. For each of 3 robots (robot_0, robot_1, robot_2):
       - robot_state_publisher  (under /{robot_name} namespace)
       - spawn_entity           (places URDF into Gazebo at staggered positions)

Robot spawn positions (spread out to avoid collisions at start):
  robot_0 :  (-8,  0) — west side
  robot_1 :  ( 0,  0) — centre
  robot_2 :  ( 8,  0) — east side

Each robot publishes:
  /{ns}/lidar/points          — sensor_msgs/PointCloud2  (3D LiDAR)
  /{ns}/camera/image_raw      — sensor_msgs/Image        (RGB)
  /{ns}/camera/depth/image_raw— sensor_msgs/Image        (Depth)
  /{ns}/imu/data              — sensor_msgs/Imu
  /{ns}/odom                  — nav_msgs/Odometry
  /{ns}/cmd_vel               — geometry_msgs/Twist  (subscribe to drive)
"""

import os
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────────────────────
# Helper: build all ROS 2 nodes for one robot
# ─────────────────────────────────────────────────────────────────────────────
def make_robot_nodes(robot_name: str, x: float, y: float, yaw: float,
                     pkg_dir: str, spawn_delay: float):
    """
    Returns a list of launch actions for one robot:
      - robot_state_publisher (immediate)
      - spawn_entity           (delayed by spawn_delay seconds to let
                                robot_state_publisher advertise the topic)
    """
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')

    # Process xacro → URDF string at launch time
    robot_urdf = xacro.process_file(
        xacro_path,
        mappings={'robot_namespace': robot_name}
    ).toxml()

    # robot_state_publisher
    #   - namespace  → all topics under /{robot_name}/
    #   - frame_prefix → TF frames named {robot_name}/base_link, etc.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_urdf,
            'frame_prefix': f'{robot_name}/',
            'use_sim_time': True,
        }],
        output='screen',
    )

    # spawn_entity: tells Gazebo to materialise the robot
    # -topic   : listens to the robot_description published above
    # -entity  : unique Gazebo model name
    # -robot_namespace : Gazebo will prepend this to internal plugin topics
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name}',
        arguments=[
            '-topic',            f'/{robot_name}/robot_description',
            '-entity',           robot_name,
            '-robot_namespace',  robot_name,
            '-x',   str(x),
            '-y',   str(y),
            '-z',   '0.12',       # just above ground
            '-Y',   str(yaw),
        ],
        output='screen',
    )

    # Wrap spawn in a timer so RSP is up before Gazebo queries the topic
    delayed_spawn = TimerAction(
        period=spawn_delay,
        actions=[spawn_node]
    )

    return [rsp_node, delayed_spawn]


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_slam_sim')

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo with GUI (set false for headless)'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'building_v2.world'),
        description='Path to Gazebo world file'
    )

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'gui':     LaunchConfiguration('gui'),
            'verbose': 'false',
            'pause':   'false',
        }.items()
    )

    # ── Robot definitions ─────────────────────────────────────────────
    # (name, x, y, yaw_rad, spawn_delay_s)
    # Stagger delays so Gazebo isn't flooded with simultaneous spawn requests.
    robots = [
        # robot_0: Large Bay, robot_1: Medium Room, robot_2: Small Room B
        ('robot_0', -20.0,  12.0, 0.0,      3.0),  # Large Bay
        ('robot_1',  18.0,  16.0, 3.14159,  5.0),  # Office
        ('robot_2',  18.0, -12.0, 1.5708,   7.0),  # Storage Room
    ]

    all_nodes = [use_sim_time_arg, gui_arg, world_arg, gazebo_launch]
    for (name, x, y, yaw, delay) in robots:
        all_nodes.extend(make_robot_nodes(name, x, y, yaw, pkg_dir, delay))

    return LaunchDescription(all_nodes)

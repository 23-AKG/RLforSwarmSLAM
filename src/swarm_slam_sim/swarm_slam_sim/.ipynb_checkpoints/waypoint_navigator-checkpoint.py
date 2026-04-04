#!/usr/bin/env python3
"""
waypoint_navigator.py

Follows a predefined list of (x, y) waypoints using a simple
proportional controller. Falls back to obstacle avoidance if
something blocks the path.

Each robot has a strategically designed path:

  robot_0 — REDUNDANT (bad broker candidate)
    Tight lawnmower in Large Bay only. Revisits same area
    repeatedly. Only briefly enters corridor.
    → Low map diversity, weak pose graph contribution.

  robot_1 — DIVERSE (good broker candidate)
    Office → corridor → Lab → corridor → repeat.
    Crosses corridor multiple times = many inter-robot
    loop closures with r2.
    → High connectivity, strong broker candidate.

  robot_2 — DIVERSE (good broker candidate)
    Storage → corridor → Workshop → corridor → repeat.
    Also crosses corridor frequently.
    → High connectivity, strong broker candidate.

This path design is intentional for the RL experiment:
  Baseline Swarm-SLAM always picks r0 (lowest ID) as broker.
  RL should learn r1 or r2 is a better choice.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ─────────────────────────────────────────────────────────────────────────────
# Waypoint definitions
# World bounds: x[-30,30]  y[-22.5,22.5]
# Rooms:
#   Large Bay  : x[-30, 6]  y[3.5, 22.5]   robot_0 starts (-20, 12)
#   Office     : x[ 6, 30]  y[10.5, 22.5]  robot_1 starts ( 18, 16)
#   Corridor   : x[-30, 6]  y[-3.5,  3.5]  (shared passage)
#   Lab        : x[ 6, 30]  y[-3.5, 10.5]
#   Workshop   : x[-30, 6]  y[-22.5, -3.5]
#   Storage    : x[ 6, 30]  y[-22.5, -3.5] robot_2 starts ( 18,-12)
#
# Door positions (gaps in walls):
#   Bay→Corridor : x ~ -10.25 to -7.75,  y = 3.5   (centre x = -9)
#   Bay→Corridor : x ~   7.75 to 10.25   via right side
#   Office→Lab   : x =  6,  y = 14..16   (door in vdiv_N)
#   Lab→Storage  : x =  6,  y = -7.5..-10 (door in vdiv_S)
# ─────────────────────────────────────────────────────────────────────────────

WAYPOINTS = {

    # ── robot_0 : REDUNDANT lawnmower in Large Bay ───────────────────────
    # Sweeps Bay at y=19, 15, 11, 7  back and forth between x=-26 and x=3
    # Then briefly enters corridor (x=-9, y=0) for minimal inter-robot contact
    # Then returns and repeats — high redundancy, low new information
    'robot_0': [
        # Sweep 1 — top of bay
        (-5,  20), (-25, 20),
        # Sweep 2
        (-25, 16), (-5,  16),
        # Sweep 3
        (-5,  12), (-25, 12),
        # Sweep 4 — bottom of bay near corridor wall
        (-25,  8), (-5,   8),
        # Brief corridor entry — through door at x=-9
        (-9,   3), (-9,   0), (-9,  -3),
        # Return to bay
        (-9,   3), (-5,   8),
        # Repeat sweep (redundancy is intentional)
        (-5,  12), (-25, 12),
        (-25, 16), (-5,  16),
        (-5,  20), (-25, 20),
        (-25, 16), (-5,  16),
        (-5,  12), (-25, 12),
        (-25,  8), (-5,   8),
        # Another brief corridor visit
        (-9,   3), (-9,   0), (-9,  -3),
        (-9,   3), (-5,   8),
    ],

    # ── robot_1 : DIVERSE Office → Corridor → Lab ────────────────────────
    # Sweeps Office, drops through door into Lab via x=6 corridor,
    # traverses the full corridor, then back. Repeats.
    # High diversity: 3 rooms + corridor visited.
    'robot_1': [
        # Office sweep (x[6,30], y[10.5,22.5])
        ( 25,  21), ( 10,  21),
        ( 10,  17), ( 25,  17),
        ( 25,  13), ( 10,  13),
        # Descend through Office/Lab door (vdiv gap at y~14-16, x=6)
        # approach door from office side
        (  8,  12),
        # Enter Lab (x[6,30], y[-3.5,10.5])
        ( 10,   8), ( 25,   8),
        ( 25,   4), ( 10,   4),
        ( 10,   0), ( 25,   0),
        ( 25,  -2), ( 10,  -2),
        # Enter corridor through Lab/Corridor wall
        # door at x=6 side, go to corridor (y~0)
        (  8,   0),
        # Traverse full corridor west to east
        ( -5,   0), (-15,   0), (-25,   0),
        # Come back east
        (-15,   0), ( -5,   0), (  8,   0),
        # Back into Lab
        ( 10,   4), ( 25,   4),
        ( 25,   8), ( 10,   8),
        # Back up to Office
        (  8,  12),
        ( 10,  13), ( 25,  13),
        ( 25,  17), ( 10,  17),
        ( 10,  21), ( 25,  21),
        # Corridor sweep again (creates more loop closures)
        (  8,  12), (  8,   0),
        (-10,   0), (-25,   0), (-10,   0), (  8,   0),
    ],

    # ── robot_2 : DIVERSE Storage → Corridor → Workshop ──────────────────
    # Mirrors robot_1's strategy on the south side.
    # Storage, corridor, Workshop. High diversity.
    'robot_2': [
        # Storage sweep (x[6,30], y[-22.5,-3.5])
        ( 10,  -5), ( 25,  -5),
        ( 25, -10), ( 10, -10),
        ( 10, -15), ( 25, -15),
        ( 25, -20), ( 10, -20),
        # Move to corridor through Lab/Storage door (x=6 gap at y=-7.5 to -10)
        (  8,  -8),
        # Enter corridor
        (  8,   0),
        # Traverse corridor west
        ( -5,   0), (-15,   0), (-25,   0),
        # Come back
        (-15,   0), ( -5,   0), (  8,   0),
        # Enter Workshop (x[-30,6], y[-22.5,-3.5]) via corridor south wall
        # door at x ~ -9, y = -3.5
        ( -9,  -3), ( -9,  -5),
        # Workshop sweep
        (-25,  -5), (-25, -10),
        ( -5, -10), ( -5, -15),
        (-25, -15), (-25, -20),
        ( -5, -20),
        # Back to corridor
        ( -9,  -5), ( -9,  -3), (  8,   0),
        # Back to storage
        (  8,  -8),
        ( 10, -10), ( 25, -10),
        ( 25,  -5), ( 10,  -5),
        # Another corridor run (more loop closures with r1)
        (  8,   0), (-10,   0), (-25,   0),
        (-10,   0), (  8,   0),
    ],
}


# ─────────────────────────────────────────────────────────────────────────────
# Navigator node
# ─────────────────────────────────────────────────────────────────────────────

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('waypoint_tolerance', 0.6)   # m — close enough
        self.declare_parameter('cruise_speed',       0.45)  # m/s
        self.declare_parameter('turn_speed',         0.8)   # rad/s
        self.declare_parameter('repeat_path',        True)  # loop forever

        ns                  = self.get_parameter('robot_name').value
        self.ns             = ns
        self.tolerance      = self.get_parameter('waypoint_tolerance').value
        self.cruise         = self.get_parameter('cruise_speed').value
        self.turn_spd       = self.get_parameter('turn_speed').value
        self.repeat         = self.get_parameter('repeat_path').value

        self.waypoints      = WAYPOINTS.get(ns, [])
        self.wp_index       = 0
        self.x = self.y = self.yaw = 0.0
        self.odom_received  = False

        # Obstacle avoidance state
        self.front_dist     = 99.0
        self.avoiding       = False
        self.avoid_timer    = 0.0

        self.get_logger().info(
            f'{ns}: Waypoint navigator starting — '
            f'{len(self.waypoints)} waypoints, repeat={self.repeat}')

        self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, f'/{ns}/odom', self.odom_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.lidar_sub = self.create_subscription(
            PointCloud2, f'/{ns}/lidar/points', self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.control_loop)

    # ── Odometry callback ─────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_received = True

    # ── LiDAR callback ────────────────────────────────────────────────────
    def lidar_callback(self, msg):
        front_min = 99.0
        try:
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True):
                x, y, z = p
                if z < -0.05 or z > 2.5:
                    continue
                dist = math.sqrt(x * x + y * y)
                if dist < 0.15:
                    continue
                angle = math.atan2(y, x)
                if -0.44 <= angle <= 0.44:   # ±25° front sector
                    front_min = min(front_min, dist)
        except Exception:
            pass
        self.front_dist = front_min

    # ── Control loop ──────────────────────────────────────────────────────
    def control_loop(self):
        if not self.odom_received or not self.waypoints:
            return

        dt = 0.1
        cmd = Twist()

        # ── Obstacle avoidance override ───────────────────────────────────
        if self.front_dist < 0.5:
            # Emergency: reverse and turn
            cmd.linear.x  = -0.1
            cmd.angular.z =  self.turn_spd
            self.cmd_pub.publish(cmd)
            self.avoiding = True
            self.avoid_timer = 1.5
            return

        if self.avoiding:
            self.avoid_timer -= dt
            if self.avoid_timer > 0:
                cmd.linear.x  = 0.05
                cmd.angular.z = self.turn_spd
                self.cmd_pub.publish(cmd)
                return
            else:
                self.avoiding = False

        # ── Waypoint following ────────────────────────────────────────────
        if self.wp_index >= len(self.waypoints):
            if self.repeat:
                self.wp_index = 0
                self.get_logger().info(f'{self.ns}: Path complete — repeating')
            else:
                self.get_logger().info(f'{self.ns}: All waypoints reached')
                self.cmd_pub.publish(Twist())
                return

        tx, ty = self.waypoints[self.wp_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Reached waypoint?
        if dist < self.tolerance:
            self.get_logger().info(
                f'{self.ns}: Reached waypoint {self.wp_index} '
                f'({tx:.1f}, {ty:.1f})')
            self.wp_index += 1
            return

        # Heading error
        target_yaw  = math.atan2(dy, dx)
        yaw_err     = target_yaw - self.yaw
        # Normalise to [-pi, pi]
        while yaw_err >  math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi

        # Turn in place if facing wrong way
        if abs(yaw_err) > 0.4:
            cmd.linear.x  = 0.05
            cmd.angular.z = self.turn_spd * (1.0 if yaw_err > 0 else -1.0)
        else:
            # Proportional speed — slow down near waypoint
            speed = min(self.cruise, self.cruise * dist / 2.0)
            speed = max(speed, 0.15)
            # Soft obstacle slowdown
            if self.front_dist < 1.2:
                speed *= (self.front_dist / 1.2)
            cmd.linear.x  = speed
            cmd.angular.z = 1.5 * yaw_err   # proportional heading correction

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
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

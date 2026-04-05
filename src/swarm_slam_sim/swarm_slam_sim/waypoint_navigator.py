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

# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs_py.point_cloud2 as pc2


# # ─────────────────────────────────────────────────────────────────────────────
# # Waypoint definitions
# # World bounds: x[-30,30]  y[-22.5,22.5]
# # Rooms:
# #   Large Bay  : x[-30, 6]  y[3.5, 22.5]   robot_0 starts (-20, 12)
# #   Office     : x[ 6, 30]  y[10.5, 22.5]  robot_1 starts ( 18, 16)
# #   Corridor   : x[-30, 6]  y[-3.5,  3.5]  (shared passage)
# #   Lab        : x[ 6, 30]  y[-3.5, 10.5]
# #   Workshop   : x[-30, 6]  y[-22.5, -3.5]
# #   Storage    : x[ 6, 30]  y[-22.5, -3.5] robot_2 starts ( 18,-12)
# #
# # Door positions (gaps in walls):
# #   Bay→Corridor : x ~ -10.25 to -7.75,  y = 3.5   (centre x = -9)
# #   Bay→Corridor : x ~   7.75 to 10.25   via right side
# #   Office→Lab   : x =  6,  y = 14..16   (door in vdiv_N)
# #   Lab→Storage  : x =  6,  y = -7.5..-10 (door in vdiv_S)
# # ─────────────────────────────────────────────────────────────────────────────

# WAYPOINTS = {

#     # ── robot_0 : REDUNDANT lawnmower in Large Bay ───────────────────────
#     # Sweeps Bay at y=19, 15, 11, 7  back and forth between x=-26 and x=3
#     # Then briefly enters corridor (x=-9, y=0) for minimal inter-robot contact
#     # Then returns and repeats — high redundancy, low new information
#     'robot_0': [
#         # Sweep 1 — top of bay
#         (-5,  20), (-25, 20),
#         # Sweep 2
#         (-25, 16), (-5,  16),
#         # Sweep 3
#         (-5,  12), (-25, 12),
#         # Sweep 4 — bottom of bay near corridor wall
#         (-25,  8), (-5,   8),
#         # Brief corridor entry — through door at x=-9
#         (-9,   3), (-9,   0), (-9,  -3),
#         # Return to bay
#         (-9,   3), (-5,   8),
#         # Repeat sweep (redundancy is intentional)
#         (-5,  12), (-25, 12),
#         (-25, 16), (-5,  16),
#         (-5,  20), (-25, 20),
#         (-25, 16), (-5,  16),
#         (-5,  12), (-25, 12),
#         (-25,  8), (-5,   8),
#         # Another brief corridor visit
#         (-9,   3), (-9,   0), (-9,  -3),
#         (-9,   3), (-5,   8),
#     ],

#     # ── robot_1 : DIVERSE Office → Corridor → Lab ────────────────────────
#     # Sweeps Office, drops through door into Lab via x=6 corridor,
#     # traverses the full corridor, then back. Repeats.
#     # High diversity: 3 rooms + corridor visited.
#     'robot_1': [
#         # Office sweep (x[6,30], y[10.5,22.5])
#         ( 25,  21), ( 10,  21),
#         ( 10,  17), ( 25,  17),
#         ( 25,  13), ( 10,  13),
#         # Descend through Office/Lab door (vdiv gap at y~14-16, x=6)
#         # approach door from office side
#         (  8,  12),
#         # Enter Lab (x[6,30], y[-3.5,10.5])
#         ( 10,   8), ( 25,   8),
#         ( 25,   4), ( 10,   4),
#         ( 10,   0), ( 25,   0),
#         ( 25,  -2), ( 10,  -2),
#         # Enter corridor through Lab/Corridor wall
#         # door at x=6 side, go to corridor (y~0)
#         (  8,   0),
#         # Traverse full corridor west to east
#         ( -5,   0), (-15,   0), (-25,   0),
#         # Come back east
#         (-15,   0), ( -5,   0), (  8,   0),
#         # Back into Lab
#         ( 10,   4), ( 25,   4),
#         ( 25,   8), ( 10,   8),
#         # Back up to Office
#         (  8,  12),
#         ( 10,  13), ( 25,  13),
#         ( 25,  17), ( 10,  17),
#         ( 10,  21), ( 25,  21),
#         # Corridor sweep again (creates more loop closures)
#         (  8,  12), (  8,   0),
#         (-10,   0), (-25,   0), (-10,   0), (  8,   0),
#     ],

#     # ── robot_2 : DIVERSE Storage → Corridor → Workshop ──────────────────
#     # Mirrors robot_1's strategy on the south side.
#     # Storage, corridor, Workshop. High diversity.
#     'robot_2': [
#         # Storage sweep (x[6,30], y[-22.5,-3.5])
#         ( 10,  -5), ( 25,  -5),
#         ( 25, -10), ( 10, -10),
#         ( 10, -15), ( 25, -15),
#         ( 25, -20), ( 10, -20),
#         # Move to corridor through Lab/Storage door (x=6 gap at y=-7.5 to -10)
#         (  8,  -8),
#         # Enter corridor
#         (  8,   0),
#         # Traverse corridor west
#         ( -5,   0), (-15,   0), (-25,   0),
#         # Come back
#         (-15,   0), ( -5,   0), (  8,   0),
#         # Enter Workshop (x[-30,6], y[-22.5,-3.5]) via corridor south wall
#         # door at x ~ -9, y = -3.5
#         ( -9,  -3), ( -9,  -5),
#         # Workshop sweep
#         (-25,  -5), (-25, -10),
#         ( -5, -10), ( -5, -15),
#         (-25, -15), (-25, -20),
#         ( -5, -20),
#         # Back to corridor
#         ( -9,  -5), ( -9,  -3), (  8,   0),
#         # Back to storage
#         (  8,  -8),
#         ( 10, -10), ( 25, -10),
#         ( 25,  -5), ( 10,  -5),
#         # Another corridor run (more loop closures with r1)
#         (  8,   0), (-10,   0), (-25,   0),
#         (-10,   0), (  8,   0),
#     ],
# }


# # ─────────────────────────────────────────────────────────────────────────────
# # Navigator node
# # ─────────────────────────────────────────────────────────────────────────────

# class WaypointNavigator(Node):

#     def __init__(self):
#         super().__init__('waypoint_navigator')

#         self.declare_parameter('robot_name', 'robot_0')
#         self.declare_parameter('waypoint_tolerance', 0.6)   # m — close enough
#         self.declare_parameter('cruise_speed',       0.45)  # m/s
#         self.declare_parameter('turn_speed',         0.8)   # rad/s
#         self.declare_parameter('repeat_path',        True)  # loop forever

#         ns                  = self.get_parameter('robot_name').value
#         self.ns             = ns
#         self.tolerance      = self.get_parameter('waypoint_tolerance').value
#         self.cruise         = self.get_parameter('cruise_speed').value
#         self.turn_spd       = self.get_parameter('turn_speed').value
#         self.repeat         = self.get_parameter('repeat_path').value

#         self.waypoints      = WAYPOINTS.get(ns, [])
#         self.wp_index       = 0
#         self.x = self.y = self.yaw = 0.0
#         self.odom_received  = False

#         # Obstacle avoidance state
#         self.front_dist     = 99.0
#         self.avoiding       = False
#         self.avoid_timer    = 0.0

#         self.get_logger().info(
#             f'{ns}: Waypoint navigator starting — '
#             f'{len(self.waypoints)} waypoints, repeat={self.repeat}')

#         self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

#         self.odom_sub = self.create_subscription(
#             Odometry, f'/{ns}/odom', self.odom_callback,
#             rclpy.qos.qos_profile_sensor_data)

#         self.lidar_sub = self.create_subscription(
#             PointCloud2, f'/{ns}/lidar/points', self.lidar_callback,
#             rclpy.qos.qos_profile_sensor_data)

#         self.timer = self.create_timer(0.1, self.control_loop)

#     # ── Odometry callback ─────────────────────────────────────────────────
#     def odom_callback(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         # yaw from quaternion
#         siny = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.yaw = math.atan2(siny, cosy)
#         self.odom_received = True

#     # ── LiDAR callback ────────────────────────────────────────────────────
#     def lidar_callback(self, msg):
#         front_min = 99.0
#         try:
#             for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
#                                      skip_nans=True):
#                 x, y, z = p
#                 if z < -0.05 or z > 2.5:
#                     continue
#                 dist = math.sqrt(x * x + y * y)
#                 if dist < 0.15:
#                     continue
#                 angle = math.atan2(y, x)
#                 if -0.44 <= angle <= 0.44:   # ±25° front sector
#                     front_min = min(front_min, dist)
#         except Exception:
#             pass
#         self.front_dist = front_min

#     # ── Control loop ──────────────────────────────────────────────────────
#     def control_loop(self):
#         if not self.odom_received or not self.waypoints:
#             return

#         dt = 0.1
#         cmd = Twist()

#         # ── Obstacle avoidance override ───────────────────────────────────
#         if self.front_dist < 0.5:
#             # Emergency: reverse and turn
#             cmd.linear.x  = -0.1
#             cmd.angular.z =  self.turn_spd
#             self.cmd_pub.publish(cmd)
#             self.avoiding = True
#             self.avoid_timer = 1.5
#             return

#         if self.avoiding:
#             self.avoid_timer -= dt
#             if self.avoid_timer > 0:
#                 cmd.linear.x  = 0.05
#                 cmd.angular.z = self.turn_spd
#                 self.cmd_pub.publish(cmd)
#                 return
#             else:
#                 self.avoiding = False

#         # ── Waypoint following ────────────────────────────────────────────
#         if self.wp_index >= len(self.waypoints):
#             if self.repeat:
#                 self.wp_index = 0
#                 self.get_logger().info(f'{self.ns}: Path complete — repeating')
#             else:
#                 self.get_logger().info(f'{self.ns}: All waypoints reached')
#                 self.cmd_pub.publish(Twist())
#                 return

#         tx, ty = self.waypoints[self.wp_index]
#         dx = tx - self.x
#         dy = ty - self.y
#         dist = math.sqrt(dx * dx + dy * dy)

#         # Reached waypoint?
#         if dist < self.tolerance:
#             self.get_logger().info(
#                 f'{self.ns}: Reached waypoint {self.wp_index} '
#                 f'({tx:.1f}, {ty:.1f})')
#             self.wp_index += 1
#             return

#         # Heading error
#         target_yaw  = math.atan2(dy, dx)
#         yaw_err     = target_yaw - self.yaw
#         # Normalise to [-pi, pi]
#         while yaw_err >  math.pi: yaw_err -= 2 * math.pi
#         while yaw_err < -math.pi: yaw_err += 2 * math.pi

#         # Turn in place if facing wrong way
#         if abs(yaw_err) > 0.4:
#             cmd.linear.x  = 0.05
#             cmd.angular.z = self.turn_spd * (1.0 if yaw_err > 0 else -1.0)
#         else:
#             # Proportional speed — slow down near waypoint
#             speed = min(self.cruise, self.cruise * dist / 2.0)
#             speed = max(speed, 0.15)
#             # Soft obstacle slowdown
#             if self.front_dist < 1.2:
#                 speed *= (self.front_dist / 1.2)
#             cmd.linear.x  = speed
#             cmd.angular.z = 1.5 * yaw_err   # proportional heading correction

#         self.cmd_pub.publish(cmd)

#     def stop(self):
#         self.cmd_pub.publish(Twist())


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ─────────────────────────────────────────────────────────────────────────────
# WAYPOINTS (with smoothing + safer door transitions + corridor offsets)
# ─────────────────────────────────────────────────────────────────────────────
WAYPOINTS = {

    # ── robot_0 : REDUNDANT ───────────────────────────────────────────────
    # 'robot_0': [
    #     # SAFE BAY REGION ONLY (no upper clutter)
    #     (-20, 12), (-8, 12),
    #     (-8, 10), (-20, 10),
    #     (-20, 8), (-8, 8),
    
    #     # Minimal corridor touch (for realism)
    #     (-10.75, 5.5),
    #     (-10.75, 3.6),
    #     (-10.75, 1.5),
    #     (-10.75, 3.6),
    #     (-10.75, 5.5),
    
    #     # Repeat same area (redundancy)
    #     (-20, 8), (-8, 8),
    #     (-8, 10), (-20, 10),
    #     (-20, 12), (-8, 12),
    # ],
    'robot_0': [
        (  -20.0,    12.0),
        (  -18.5,    12.0),
        (  -17.0,    12.0),
        (  -15.9,    13.1),
        (  -15.4,    14.5),
        (  -15.0,    16.0),
        (  -14.1,    17.2),
        (  -12.6,    17.7),
        (  -11.2,    18.2),
        (   -9.8,    18.8),
        (   -8.3,    18.3),
        (   -6.8,    18.3),
        (   -5.6,    19.2),
        (   -4.1,    19.2),
        (   -3.2,    18.0),
        (   -1.9,    18.8),
        (   -0.7,    19.7),
        (    0.7,    19.0),
        (    2.1,    18.4),
        (    2.1,    16.9),
        (    1.8,    15.4),
        (    1.9,    13.9),
        (    2.5,    12.5),
        (    3.1,    11.1),
        (    3.5,     9.6),
        (    2.9,     8.3),
        (    2.3,     6.9),
        (    2.1,     5.4),
        (    1.7,     3.9),
        (    2.1,     2.5),
        (    2.7,     1.1),
        (    3.4,    -0.2),
        (    2.8,    -1.6),
        (    2.3,    -3.0),
        (    2.6,    -4.5),
        (    3.6,    -5.7),
        (    4.6,    -6.8),
        (    4.2,    -8.3),
        (    3.5,    -9.6),
        (    2.9,   -11.0),
        (    2.6,   -12.5),
        (    3.1,   -13.9),
        (    3.5,   -15.3),
        (    3.9,   -16.8),
        (    4.4,   -18.2),
        (    3.6,   -19.5),
        (    2.1,   -19.7),
        (    0.6,   -19.3),
        (   -0.7,   -18.6),
        (   -2.0,   -17.8),
        (   -3.3,   -17.0),
        (   -4.7,   -17.5),
        (   -6.0,   -18.3),
        (   -7.3,   -19.0),
        (   -8.6,   -19.8),
        (  -10.1,   -19.9),
        (  -11.6,   -19.5),
        (  -13.1,   -19.1),
        (  -14.6,   -18.6),
        (  -16.0,   -18.2),
        (  -17.5,   -17.8),
        (  -18.3,   -16.5),
        (  -19.0,   -15.2),
        (  -19.7,   -13.8),
        (  -20.5,   -12.4),
        (  -21.2,   -11.1),
        (  -21.9,    -9.7),
        (  -22.6,    -8.3),
        (  -23.3,    -7.0),
        (  -24.0,    -5.6),
        (  -24.7,    -4.3),
        (  -25.4,    -3.0),
        (  -26.1,    -1.6),
        (  -26.8,    -0.3),
        (  -27.5,     1.0),
        (  -28.2,     2.4),
        (  -28.9,     3.7),
        (  -29.6,     5.0),
        (  -30.3,     6.4),
        (  -31.1,     7.7),
        (  -29.9,     8.7),
        (  -28.4,     8.9),
        (  -26.9,     9.1),
        (  -25.4,     9.3),
        (  -23.9,     9.5),
        (  -22.4,     9.7),
        (  -21.1,    10.3),
        (  -20.5,    11.7),
        (  -19.2,    12.5),
    ],
    # ── robot_1 : DIVERSE ────────────────────────────────────────────────
    # 'robot_1': [
    #     # Office sweep (avoid desks at y≈19)
    #     (10, 21), (26, 21),
    #     (26, 17), (10, 17),
    #     (10, 13), (26, 13),
    
    #     # Move to door (safe alignment)
    #     (8, 15.25),
    #     (6.2, 15.25),
    #     (4, 15.25),
    
    #     # Bay sweep (SAFE region only)
    #     (4, 11), (-24, 11),
    #     (-24, 8), (4, 8),
    
    #     # Enter corridor
    #     (-10.75, 5.5),
    #     (-10.75, 3.6),
    #     (-10.75, 1.5),
    
    #     # Corridor traversal (SAFE OFFSET)
    #     (-10.75, 0.8),
    #     (-25, 0.8),
    #     (5, 0.8),
    
    #     # Return
    #     (-10.75, 1.5),
    #     (-10.75, 3.6),
    #     (-10.75, 5.5),
    
    #     (4, 8), (-24, 8),
    #     (-24, 11), (4, 11),
    
    #     (4, 15.25),
    #     (8, 15.25),
    
    #     (10, 13), (26, 13),
    #     (26, 17), (10, 17),
    # ],
    'robot_1': [
        (   18.1,    16.0),
        (   16.9,    16.9),
        (   16.8,    18.4),
        (   16.7,    19.9),
        (   16.6,    21.4),
        (   16.5,    22.9),
        (   15.0,    23.3),
        (   13.5,    23.2),
        (   12.0,    23.2),
        (   10.5,    23.1),
        (   10.3,    23.1),
        (   11.5,    24.0),
        (   13.0,    24.0),
        (   14.3,    24.7),
        (   15.6,    24.0),
        (   16.3,    25.4),
        (   16.8,    26.8),
        (   18.3,    27.1),
        (   19.8,    27.1),
        (   21.3,    27.1),
        (   22.9,    27.1),
        (   24.4,    27.0),
        (   25.9,    27.0),
        (   27.4,    27.0),
        (   28.9,    27.0),
        (   30.4,    26.9),
        (   31.9,    26.9),
        (   33.4,    26.8),
        (   34.6,    25.9),
        (   35.5,    24.7),
        (   36.4,    23.4),
        (   37.2,    22.1),
        (   38.0,    20.8),
        (   37.8,    19.3),
        (   37.7,    17.8),
        (   37.6,    16.3),
        (   37.5,    14.7),
        (   37.3,    13.2),
        (   37.2,    11.7),
        (   37.1,    10.1),
        (   36.9,     8.6),
        (   37.2,     7.1),
        (   37.2,     5.5),
        (   37.1,     4.0),
        (   37.0,     2.4),
        (   36.9,     0.9),
        (   36.8,    -0.6),
        (   36.8,    -2.1),
        (   36.7,    -3.6),
        (   36.6,    -5.2),
        (   36.5,    -6.6),
        (   36.4,    -8.2),
        (   36.3,    -9.7),
        (   36.2,   -11.2),
        (   35.6,   -12.6),
        (   34.4,   -13.4),
        (   33.1,   -14.3),
        (   31.9,   -15.2),
        (   31.9,   -16.7),
        (   32.0,   -18.2),
        (   31.2,   -19.5),
        (   29.8,   -20.0),
        (   28.3,   -20.5),
        (   26.9,   -20.3),
        (   25.4,   -19.9),
        (   24.0,   -20.4),
        (   22.8,   -19.5),
        (   21.3,   -19.6),
        (   20.2,   -20.7),
        (   19.0,   -19.8),
        (   18.8,   -18.3),
        (   18.3,   -16.9),
        (   17.8,   -15.4),
        (   17.6,   -13.9),
        (   17.5,   -12.4),
        (   19.0,   -11.9),
        (   19.4,   -10.5),
        (   19.2,    -9.0),
        (   19.0,    -7.5),
        (   18.8,    -6.0),
        (   18.6,    -4.5),
        (   18.4,    -3.0),
        (   18.2,    -1.5),
        (   18.0,    -0.0),
        (   18.5,     1.4),
        (   19.0,     2.8),
        (   19.5,     4.2),
        (   20.0,     5.6),
        (   20.5,     7.1),
        (   21.0,     8.5),
        (   21.6,     9.9),
        (   22.1,    11.3),
        (   21.4,    12.7),
        (   20.0,    13.3),
        (   18.6,    13.9),
        (   18.0,    15.3),
    ],


    # ── robot_2 : DIVERSE ────────────────────────────────────────────────
    # 'robot_2': [
    #     # Storage — ABOVE shelves
    #     (10, -5), (27, -5),
    
    #     # BELOW shelves
    #     (27, -14), (10, -14),
    
    #     # Lower sweep
    #     (10, -18), (24, -18),
    
    #     # Move to workshop
    #     (8, -8.75),
    #     (6.2, -8.75),
    #     (4, -8.75),
    
    #     # Workshop safe sweep
    #     (-5, -7), (-18, -7),
    #     (-18, -12), (-5, -12),
    #     (-5, -19), (-18, -19),
    
    #     # Corridor entry
    #     (-10.75, -5),
    #     (-10.75, -3.6),
    #     (-10.75, -1.5),
    
    #     # Corridor traversal (OFFSET)
    #     (-10.75, -0.8),
    #     (-25, -0.8),
    #     (5, -0.8),
    
    #     # Return
    #     (-10.75, -1.5),
    #     (-10.75, -3.6),
    #     (-10.75, -5),
    
    #     (-18, -12), (-5, -12),
    
    #     (4, -8.75),
    #     (8, -8.75),
    
    #     (10, -14), (27, -14),
    #     (27, -5), (10, -5),
    # ],
    'robot_2': [
        (   18.0,   -12.1),
        (   18.7,   -10.8),
        (   19.5,    -9.5),
        (   20.2,    -8.2),
        (   21.0,    -6.8),
        (   21.4,    -5.4),
        (   21.4,    -3.9),
        (   21.2,    -2.4),
        (   20.3,    -1.2),
        (   19.3,    -0.0),
        (   19.0,     1.4),
        (   19.3,     2.9),
        (   19.2,     4.4),
        (   19.0,     5.9),
        (   18.8,     7.4),
        (   18.6,     8.9),
        (   18.5,    10.4),
        (   17.6,    11.7),
        (   16.2,    12.2),
        (   14.8,    12.7),
        (   14.0,    14.0),
        (   13.8,    15.5),
        (   13.5,    16.9),
        (   12.3,    17.8),
        (   10.8,    17.5),
        (    9.6,    18.5),
        (    8.5,    19.5),
        (    7.0,    19.0),
        (    5.7,    18.4),
        (    4.3,    17.8),
        (    2.9,    18.4),
        (    1.4,    18.7),
        (    0.0,    19.2),
        (   -1.5,    19.1),
        (   -3.0,    19.7),
        (   -4.4,    20.2),
        (   -5.8,    20.8),
        (   -7.3,    20.9),
        (   -8.4,    19.9),
        (   -9.5,    18.8),
        (  -10.6,    17.8),
        (  -11.7,    16.8),
        (  -12.9,    15.7),
        (  -14.0,    14.7),
        (  -15.1,    13.6),
        (  -16.2,    12.6),
        (  -17.3,    11.5),
        (  -17.3,    10.0),
        (  -16.8,     8.6),
        (  -17.6,     7.3),
        (  -18.6,     6.2),
        (  -19.7,     5.1),
        (  -19.4,     3.6),
        (  -18.7,     2.3),
        (  -18.0,     1.0),
        (  -17.3,    -0.4),
        (  -17.6,    -1.8),
        (  -18.3,    -3.2),
        (  -19.1,    -4.5),
        (  -19.8,    -5.8),
        (  -20.0,    -7.3),
        (  -19.2,    -8.6),
        (  -18.4,    -9.8),
        (  -17.6,   -11.1),
        (  -16.8,   -12.4),
        (  -16.0,   -13.7),
        (  -15.2,   -14.9),
        (  -14.4,   -16.2),
        (  -13.5,   -17.5),
        (  -12.0,   -17.6),
        (  -10.5,   -17.7),
        (   -9.4,   -18.8),
        (   -8.3,   -19.8),
        (   -7.2,   -20.8),
        (   -6.1,   -21.9),
        (   -4.6,   -21.4),
        (   -3.1,   -21.3),
        (   -2.1,   -20.3),
        (   -0.7,   -19.5),
        (    0.8,   -19.4),
        (    2.2,   -19.3),
        (    3.8,   -19.1),
        (    5.3,   -19.0),
        (    6.8,   -18.8),
        (    8.3,   -18.7),
        (    9.4,   -19.7),
        (   10.5,   -20.8),
        (   12.0,   -20.6),
        (   13.4,   -20.0),
        (   14.7,   -19.3),
        (   15.0,   -17.8),
        (   14.9,   -16.3),
        (   14.9,   -14.8),
        (   16.2,   -13.9),
        (   17.4,   -13.0),
        (   16.0,   -12.4),
    ],

}


# ─────────────────────────────────────────────────────────────────────────────
# Navigator
# ─────────────────────────────────────────────────────────────────────────────

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('waypoint_tolerance', 0.7)
        self.declare_parameter('cruise_speed', 0.40)
        self.declare_parameter('turn_speed', 0.80)
        self.declare_parameter('repeat_path', True)

        ns = self.get_parameter('robot_name').value
        self.ns = ns
        self.tol = self.get_parameter('waypoint_tolerance').value
        self.cruise = self.get_parameter('cruise_speed').value
        self.turn_spd = self.get_parameter('turn_speed').value
        self.repeat = self.get_parameter('repeat_path').value

        self.waypoints = WAYPOINTS.get(ns, [])
        self.wp_idx = 0
        self.x = self.y = self.yaw = 0.0
        self.odom_ok = False
        self.front_dist = 99.0
        self.avoiding = False
        self.avoid_t = 0.0

        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        self.create_subscription(
            Odometry, f'/{ns}/odom', self._odom_cb, 10)

        self.create_subscription(
            PointCloud2, f'/{ns}/lidar/points', self._lidar_cb, 10)

        # self.create_timer(0.1, self._loop)
        self.create_timer(0.5, self._loop)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ok = True

    def _lidar_cb(self, msg):
        fmin = 99.0
        try:
            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                x, y, z = p
                if z < -0.05 or z > 2.5:
                    continue
                d = math.sqrt(x*x + y*y)
                if d < 0.15:
                    continue
                if -0.44 <= math.atan2(y, x) <= 0.44:
                    fmin = min(fmin, d)
        except Exception:
            pass
        self.front_dist = fmin

    def _loop(self):
        if not self.odom_ok or not self.waypoints:
            return

        cmd = Twist()
        dt = 0.1

        # EARLIER avoidance trigger
        if self.front_dist < 0.6:
            cmd.linear.x = -0.1
            cmd.angular.z = self.turn_spd
            self.pub.publish(cmd)
            self.avoiding = True
            self.avoid_t = 1.5
            return

        if self.avoiding:
            self.avoid_t -= dt
            if self.avoid_t > 0:
                cmd.linear.x = 0.05
                cmd.angular.z = self.turn_spd
                self.pub.publish(cmd)
                return
            self.avoiding = False

        if self.wp_idx >= len(self.waypoints):
            if self.repeat:
                self.wp_idx = 0
            else:
                self.pub.publish(Twist())
                return

        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.tol:
            self.wp_idx += 1
            return

        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self.yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        if abs(err) > 0.5:
            cmd.linear.x = 0.05
            cmd.angular.z = self.turn_spd * (1 if err > 0 else -1)
        else:
            # spd = min(self.cruise, self.cruise * dist / 1.5)
            spd = self.cruise
            spd = max(spd, 0.08)

            if self.front_dist < 1.5:
                spd *= max(0.3, (self.front_dist - 0.6) / 0.9)

            cmd.linear.x = spd
            cmd.angular.z = 1.8 * err

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
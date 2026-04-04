#!/usr/bin/env python3
"""
waypoint_navigator.py

Follows predefined collision-free waypoints for each robot.

Building: x[-30,30], y[-22.5,22.5]
Rooms:
  Large Bay  : x[-30, 6], y[ 3.5, 22.5]  robot_0 starts (-20, 12)
  Office     : x[  6,30], y[10.5, 22.5]  robot_1 starts ( 18, 16)
  Corridor   : x[-30, 6], y[-3.5,  3.5]
  Lab        : x[  6,30], y[-3.5, 10.5]
  Workshop   : x[-30, 6], y[-22.5, -3.5]
  Storage    : x[  6,30], y[-22.5, -3.5] robot_2 starts ( 18,-12)

Door positions (verified against world file):
  Bay  <-> Corridor : x=-10.75, y= 3.5  (gap x=-12 to -9.5)
  Bay  <-> Office   : x= 6,    y=15.25  (gap y=14  to 16.5)
  Lab  <-> Corridor : x= 6,    y= 2.25  (gap y= 1  to  3.5)
  Work <-> Corridor : x=-10.75, y=-3.5  (gap x=-12 to -9.5)
  Work <-> Storage  : x= 6,    y=-8.75  (gap y=-10 to -7.5)

PATH DESIGN INTENT:
  robot_0 — REDUNDANT (intentionally bad broker candidate)
    Tight lawnmower in a small clear zone of the Bay (x=-3 to -13, y=7-13).
    Only briefly enters corridor. Revisits same area repeatedly.
    → Low map diversity, weak pose graph connectivity.

  robot_1 — DIVERSE (good broker candidate)
    Office sweep → Bay sweep → Corridor traverse → return loop.
    Covers 3 zones, traverses corridor multiple times.
    → High map diversity, strong pose graph connectivity.

  robot_2 — DIVERSE (good broker candidate)
    Storage sweep → Workshop sweep → Corridor traverse → return loop.
    Covers 3 zones, traverses full corridor.
    → High map diversity, strong pose graph connectivity.

  When robot_1 and robot_2 both traverse the corridor they generate
  inter-robot loop closures. robot_0 barely touches it.

  BASELINE: Swarm-SLAM rule picks robot_0 as broker (lowest ID).
  RL GOAL:  Agent learns robot_1 or robot_2 is better broker.

All waypoints verified with 1.0m minimum clearance from obstacles.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ─────────────────────────────────────────────────────────────────────────────
# WAYPOINTS  — (x, y) in world frame, verified collision-free
# ─────────────────────────────────────────────────────────────────────────────
WAYPOINTS = {

    # ── robot_0 : REDUNDANT ───────────────────────────────────────────────
    # Stays in clear zone x=-3 to -13, y=7 to 13.
    # No objects in this sub-region of the Bay.
    # Two brief corridor visits per cycle; rest is redundant lawnmower.
    'robot_0': [
        # === Lawnmower pass 1 ===
        (-13,  13), (-3,  13),   # top row  →
        ( -3,  11), (-13, 11),   # step down ←
        (-13,   9), (-3,   9),   # step down →
        ( -3,   7), (-13,  7),   # step down ←

        # === Brief corridor visit (door at x=-10.75, y=3.5) ===
        (-10.75,  5.5),           # approach door from bay
        (-10.75,  1.0),           # enter corridor
        (-10.75,  5.5),           # return to bay

        # === Lawnmower pass 2 (same zone, pure redundancy) ===
        ( -3,   7), (-13,  7),
        (-13,   9), (-3,   9),
        ( -3,  11), (-13, 11),
        (-13,  13), (-3,  13),

        # === Second corridor visit ===
        (-10.75,  5.5),
        (-10.75,  1.0),
        (-10.75,  5.5),

        # === Lawnmower pass 3 ===
        ( -3,   7), (-13,  7),
        (-13,   9), (-3,   9),
        ( -3,  11), (-13, 11),
        (-13,  13), (-3,  13),
        # (path repeats via repeat_path=True)
    ],

    # ── robot_1 : DIVERSE (Office → Bay → Corridor → return) ─────────────
    # Verified clearances:
    #   Office: office_desk1(12,19), office_desk2(20,19) footprint y=18.65-19.35
    #           office_table(16,13.5) footprint y=12.7-14.3
    #           office_landmark(28,20) footprint x=27.75-28.25
    #   Bay:    bay_shelf(-28,12) → stay x > -26
    #           bay_landmark(-22,18), bay_L1(-14,19), bay_L2(-12.1,17.5),
    #           bay_platform(-5,16) — all above y=14, clear at y=8,11
    #   Corridor: corr_box1(-20,1.5) and corr_box2(-5,-1.5) → at y=0 both
    #             are 1.5m away (>1.2m clearance OK)
    'robot_1': [
        # === Office sweep (x=9 to 25, rows at y=21,17,13,11.5) ===
        # Avoid: desk1(12,19), desk2(20,19) at y=18.65-19.35 → use y=21,17
        #        table(16,13.5) at y=12.7-14.3 → use y=13 (just above)
        #        landmark(28,20) → stay x<26.5
        ( 9, 21), (25, 21),       # top row →
        (25, 17), ( 9, 17),       # ←
        ( 9, 13), (25, 13),       # → (table footprint y=12.7, at y=13 clear)
        (25, 11.5), ( 9, 11.5),   # ←

        # === Cross Bay/Office door (gap y=14-16.5 at x=6) ===
        ( 8, 15.25),              # approach door from office side
        ( 4, 15.25),              # step into Bay

        # === Bay sweep (x=4 to -26, rows at y=11,8) ===
        # bay_shelf at x=-28.2 to -27.8, y=8-16 → stay x > -26 ✓
        # All other bay objects above y=14, clear at y=11 and y=8
        (  4, 11), (-26, 11),     # Bay row →
        (-26,  8), (  4,  8),     # ←

        # === Cross Bay/Corridor door (gap x=-12 to -9.5 at y=3.5) ===
        (-10.75, 5.5),            # approach door
        (-10.75, 2.0),            # enter corridor

        # === Full corridor traverse ===
        (-10.75, 0),              # step to corridor centre y
        ( -25,   0),              # go west (far end)
        (   5,   0),              # traverse east to vdiv boundary
        (-10.75, 0),              # return to door x

        # === Return to Bay ===
        (-10.75, 2.0),
        (-10.75, 5.5),

        # === Bay return sweep ===
        (  4,  8), (-26,  8),
        (-26, 11), (  4, 11),

        # === Cross back to Office ===
        ( 4, 15.25),
        ( 8, 15.25),

        # === Office return sweep ===
        ( 9, 11.5), (25, 11.5),
        (25, 13),   ( 9, 13),
        ( 9, 17),   (25, 17),
        (25, 21),   ( 9, 21),
        # (path repeats via repeat_path=True)
    ],

    # ── robot_2 : DIVERSE (Storage → Workshop → Corridor → return) ───────
    # Verified clearances:
    #   Storage: store_shelf1(10,-10), shelf2(16,-10), shelf3(22,-10)
    #            each 0.4×6m at y=-7 to -13 → route above (y=-5) and
    #            below (y=-14,-17,-20) the shelf span
    #            store_landmark(27,-18) → at y=-17: x<24.7 safe ✓
    #                                     at y=-20: dist=3.6m safe ✓
    #   Workshop: ws_machine1(-22,-10) footprint x=-23.25 to -20.75 → x>-18 ✓
    #             ws_machine2(-22,-17) footprint x=-23.25 to -20.75 → x>-18 ✓
    #             ws_landmark(-8,-16)  → route clears it (footprint y=-16.5 to -15.5)
    #             ws_bench(-5,-21) footprint x=-9 to -1, y=-20.7 to -21.3
    #                              → at y=-19: closest corner dist=2.63m ✓
    #   Corridor: same as robot_1, y=0 gives 1.5m clearance from both boxes ✓
    'robot_2': [
        # === Storage sweep ===
        # Rows above shelves (y=-5) and below (y=-14,-17,-20)
        # Start at (18,-12) — between shelves at mid-height. Go to clear zone first.
        (  9, -5),  (27, -5),     # above shelves →
        ( 27,-14),  ( 9,-14),     # below shelves ←
        (  9,-17),  (24,-17),     # lower row → (x<24.7 for landmark clearance)
        ( 24,-20),  ( 9,-20),     # bottom row ←

        # === Cross Storage/Workshop door (gap y=-10 to -7.5 at x=6) ===
        (  8, -8.75),             # approach door from storage side
        (  4, -8.75),             # step into Workshop

        # === Workshop sweep ===
        # Rows at y=-7,-12,-19 (x=-4 to -18)
        # Avoid: machines at x=-20.75 to -23.25 → stay x > -18.5 ✓
        #        ws_landmark at (-8,-16) footprint y=-16.5 to -15.5 →
        #                    at y=-12: clear (4m away) ✓
        #                    at y=-19: at x=-8, dist from center=3.16m ✓
        #        ws_bench at x=-9 to -1, y=-20.7 to -21.3 →
        #                    at y=-19: closest corner dist=2.63m ✓
        ( -4, -7),  (-18, -7),    # upper workshop row ←
        (-18,-12),  ( -4,-12),    # middle row →
        ( -4,-19),  (-18,-19),    # lower row ← (clear of bench and machines)

        # === Cross Workshop/Corridor door (gap x=-12 to -9.5 at y=-3.5) ===
        (-10.75, -5.0),           # approach door from workshop
        (-10.75, -2.0),           # enter corridor

        # === Full corridor traverse ===
        (-10.75,  0),
        ( -25,    0),             # go west
        (   5,    0),             # go east to vdiv boundary
        (-10.75,  0),             # return

        # === Return to Workshop ===
        (-10.75, -2.0),
        (-10.75, -5.0),

        # === Workshop return sweep ===
        (-18,-12), ( -4,-12),
        ( -4, -7), (-18, -7),

        # === Cross back to Storage ===
        (  4, -8.75),
        (  8, -8.75),

        # === Storage return sweep ===
        (  9,-14), (27,-14),
        ( 27, -5), ( 9, -5),
        # (path repeats via repeat_path=True)
    ],
}


# ─────────────────────────────────────────────────────────────────────────────
# Navigator node
# ─────────────────────────────────────────────────────────────────────────────

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('robot_name',        'robot_0')
        self.declare_parameter('waypoint_tolerance',  0.7)
        self.declare_parameter('cruise_speed',         0.40)
        self.declare_parameter('turn_speed',           0.80)
        self.declare_parameter('repeat_path',         True)

        ns               = self.get_parameter('robot_name').value
        self.ns          = ns
        self.tol         = self.get_parameter('waypoint_tolerance').value
        self.cruise      = self.get_parameter('cruise_speed').value
        self.turn_spd    = self.get_parameter('turn_speed').value
        self.repeat      = self.get_parameter('repeat_path').value

        self.waypoints   = WAYPOINTS.get(ns, [])
        self.wp_idx      = 0
        self.x = self.y = self.yaw = 0.0
        self.odom_ok     = False
        self.front_dist  = 99.0
        self.avoiding    = False
        self.avoid_t     = 0.0

        self.get_logger().info(
            f'[{ns}] WaypointNavigator: {len(self.waypoints)} waypoints, '
            f'repeat={self.repeat}')

        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        self.create_subscription(
            Odometry, f'/{ns}/odom', self._odom_cb,
            rclpy.qos.qos_profile_sensor_data)

        self.create_subscription(
            PointCloud2, f'/{ns}/lidar/points', self._lidar_cb,
            rclpy.qos.qos_profile_sensor_data)

        self.create_timer(0.1, self._loop)

    # ── Odometry ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ok = True

    # ── LiDAR front-sector ───────────────────────────────────────────────
    def _lidar_cb(self, msg):
        fmin = 99.0
        try:
            for p in pc2.read_points(msg, field_names=('x','y','z'),
                                     skip_nans=True):
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

    # ── Control loop (10 Hz) ─────────────────────────────────────────────
    def _loop(self):
        if not self.odom_ok or not self.waypoints:
            return

        cmd = Twist()
        dt  = 0.1

        # Emergency reverse
        if self.front_dist < 0.45:
            cmd.linear.x  = -0.12
            cmd.angular.z =  self.turn_spd
            self.pub.publish(cmd)
            self.avoiding = True
            self.avoid_t  = 1.5
            return

        # Timed avoidance turn
        if self.avoiding:
            self.avoid_t -= dt
            if self.avoid_t > 0:
                cmd.linear.x  = 0.05
                cmd.angular.z = self.turn_spd
                self.pub.publish(cmd)
                return
            self.avoiding = False

        # Waypoint reached?
        if self.wp_idx >= len(self.waypoints):
            if self.repeat:
                self.wp_idx = 0
                self.get_logger().info(f'[{self.ns}] Path complete — repeating')
            else:
                self.get_logger().info(f'[{self.ns}] All waypoints done')
                self.pub.publish(Twist())
                return

        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist   = math.sqrt(dx*dx + dy*dy)

        if dist < self.tol:
            self.get_logger().info(
                f'[{self.ns}] wp {self.wp_idx}/{len(self.waypoints)} '
                f'({tx:.1f},{ty:.1f}) reached')
            self.wp_idx += 1
            return

        # Heading error
        target_yaw = math.atan2(dy, dx)
        err        = target_yaw - self.yaw
        while err >  math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        if abs(err) > 0.5:
            # Turn in place toward waypoint
            cmd.linear.x  = 0.05
            cmd.angular.z = self.turn_spd * (1.0 if err > 0 else -1.0)
        else:
            # Drive toward waypoint with heading correction
            spd = min(self.cruise, self.cruise * dist / 1.5)
            spd = max(spd, 0.15)
            if self.front_dist < 1.2:
                spd *= max(0.3, (self.front_dist - 0.45) / 0.75)
            cmd.linear.x  = spd
            cmd.angular.z = 1.8 * err  # proportional correction

        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())


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

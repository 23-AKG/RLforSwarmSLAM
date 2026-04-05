import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from cslam_common_interfaces.msg import (
    RobotIdsAndOrigin,
    LocalDescriptorsRequest,
    IntraRobotLoopClosure,
    InterRobotLoopClosure,
    InterRobotMatches,
)
from stable_baselines3 import PPO


class BrokerSelectorNode(Node):
    def __init__(self):
        super().__init__('broker_selector_node')

        # ── params ──────────────────────────────────────────────────
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_nb_robots', 3)
        self.declare_parameter('broker_selection_period_sec', 3.0)
        self.declare_parameter('model_path',
            '/home/srihariacharya/Swarm-SLAM/gazebo_env/broker_policy_v3_world_calibrated_2.zip')

        self.robot_id     = self.get_parameter('robot_id').value
        self.n_robots     = self.get_parameter('max_nb_robots').value
        self.model_path   = self.get_parameter('model_path').value
        period            = self.get_parameter(
                                'broker_selection_period_sec').value

        # ── state that mirrors the training obs vector ───────────────
        self.keyframe_counts       = np.zeros(self.n_robots, dtype=float)
        self.intra_closure_counts  = np.zeros(self.n_robots, dtype=float)
        self.neighbor_counts       = np.zeros(self.n_robots, dtype=float)
        self.descriptor_req_counts = np.zeros(self.n_robots, dtype=float)
        self.inter_closures_total  = 0.0
        self.matches_total         = 0.0
        self.current_broker        = 0
        self.round                 = 0
        self.max_rounds            = 50   # matches training

        # ── load policy ─────────────────────────────────────────────
        self.model = PPO.load(self.model_path)
        self.get_logger().info(f'Loaded broker policy from {self.model_path}')

        # ── per-robot subscriptions ──────────────────────────────────
        for rid in range(self.n_robots):
            ns = f'/r{rid}'

            self.create_subscription(
                IntraRobotLoopClosure,
                f'{ns}/cslam/intra_robot_loop_closure',
                lambda msg, r=rid: self._intra_cb(msg, r),
                100)

            self.create_subscription(
                RobotIdsAndOrigin,
                f'{ns}/cslam/current_neighbors',
                lambda msg, r=rid: self._neighbors_cb(msg, r),
                100)

            self.create_subscription(
                LocalDescriptorsRequest,
                f'{ns}/cslam/local_descriptors_request',
                lambda msg, r=rid: self._desc_req_cb(msg, r),
                100)

        # ── global subscriptions ─────────────────────────────────────
        self.create_subscription(
            InterRobotLoopClosure,
            '/cslam/inter_robot_loop_closure',
            self._inter_closure_cb,
            100)

        self.create_subscription(
            InterRobotMatches,
            '/cslam/inter_robot_matches',
            self._matches_cb,
            100)

        # ── publisher ────────────────────────────────────────────────
        self.broker_pub = self.create_publisher(
            Int32, '/cslam/best_broker', 100)

        # ── timer ────────────────────────────────────────────────────
        self.create_timer(period, self._select_broker)

        self.get_logger().info(
            f'BrokerSelectorNode ready '
            f'(robot_id={self.robot_id}, n_robots={self.n_robots})')

    # ── callbacks ────────────────────────────────────────────────────

    def _intra_cb(self, msg: IntraRobotLoopClosure, robot_id: int):
        self.keyframe_counts[robot_id] += 1
        if msg.success:
            self.intra_closure_counts[robot_id] += 1

    def _neighbors_cb(self, msg: RobotIdsAndOrigin, robot_id: int):
        self.neighbor_counts[robot_id] = float(len(msg.robots.ids))

    def _desc_req_cb(self, msg: LocalDescriptorsRequest, robot_id: int):
        self.descriptor_req_counts[robot_id] += 1

    def _inter_closure_cb(self, msg: InterRobotLoopClosure):
        if msg.success:
            self.inter_closures_total += 1.0

    def _matches_cb(self, msg: InterRobotMatches):
        self.matches_total += float(len(msg.matches))

    # ── broker selection ─────────────────────────────────────────────

    def _build_obs(self) -> np.ndarray:
        obs = []
        for i in range(self.n_robots):
            obs.append(min(self.keyframe_counts[i]       / 500.0,  10.0))
            obs.append(min(self.intra_closure_counts[i]  / 100.0,  10.0))
            obs.append(min(self.neighbor_counts[i]       / self.n_robots, 10.0))
            obs.append(min(self.descriptor_req_counts[i] / 200.0,  10.0))
        obs.append(min(self.inter_closures_total / 1000.0, 10.0))
        obs.append(min(self.matches_total        / 1000.0, 10.0))
        obs.append(float(self.current_broker)    / self.n_robots)
        obs.append(float(self.round)             / self.max_rounds)
        return np.array(obs, dtype=np.float32)

    def _select_broker(self):
        obs = self._build_obs()
        action, _ = self.model.predict(obs, deterministic=True)
        broker_id  = int(action)

        self.current_broker = broker_id
        self.round = min(self.round + 1, self.max_rounds - 1)

        msg = Int32()
        msg.data = broker_id
        self.broker_pub.publish(msg)

        self.get_logger().info(f'Selected broker: r{broker_id}')


def main(args=None):
    rclpy.init(args=args)
    node = BrokerSelectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

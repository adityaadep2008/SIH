import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import PeerTrackArray, RobotState

from .algorithms import avoidance_velocity, finite_command
from .common import FLEET_STATE_QOS, POSE_QOS


class OrcaNode(Node):
    """Produces an uncertainty-inflated reciprocal-velocity safety candidate."""
    def __init__(self):
        super().__init__('orca_node')
        self.radius = self.declare_parameter('robot_radius_m', 0.35).value
        self.max_speed = self.declare_parameter('max_speed_mps', 6.0).value
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.pose, self.desired, self.tracks = None, Twist(), []
        self._received_local_state = False
        self.pub = self.create_publisher(Twist, 'cmd_vel_candidate', FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(Twist, 'cmd_vel_desired', lambda msg: setattr(self, 'desired', msg), FLEET_STATE_QOS)
        self.create_subscription(PeerTrackArray, 'peer_tracks', lambda msg: setattr(self, 'tracks', msg.tracks), FLEET_STATE_QOS)
        self.create_timer(0.1, self.control)

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose = msg.pose

    def on_local_state(self, msg):
        if msg.localization_valid:
            self.pose = msg.pose
            if not self._received_local_state:
                self._received_local_state = True
                self.get_logger().info('ORCA received first local RobotState sample')

    def control(self):
        result = Twist()
        if self.pose is not None and finite_command(self.desired.linear.x, self.desired.angular.z):
            direction = (math.cos(self.pose.theta), math.sin(self.pose.theta))
            preferred = (self.desired.linear.x * direction[0], self.desired.linear.x * direction[1])
            peers = [
                {'x': p.pose.x, 'y': p.pose.y, 'vx': p.twist.linear.x, 'vy': p.twist.linear.y,
                 'radius_inflation': 2.0 * math.sqrt(max(p.covariance_trace, 0.0)),
                 'id': getattr(p, 'robot_id', '')}
                for p in self.tracks
                if all(math.isfinite(value) for value in (
                    p.pose.x, p.pose.y, p.twist.linear.x, p.twist.linear.y, p.covariance_trace))
            ]
            vx, vy = avoidance_velocity(preferred, (self.pose.x, self.pose.y), peers, self.radius, 1.5, self.max_speed, self_id=self.robot_id)
            raw_linear_x = vx * direction[0] + vy * direction[1]
            if self.desired.linear.x < -0.01:
                result.linear.x = max(-self.max_speed, self.desired.linear.x)
            elif self.desired.linear.x <= 0.01:
                result.linear.x = 0.0
            else:
                result.linear.x = max(0.0, min(raw_linear_x, self.desired.linear.x))
            result.angular.z = self.desired.angular.z
            if peers and abs(result.linear.x - self.desired.linear.x) > 0.05:
                self.get_logger().info(
                    f'[{self.robot_id}:ORCA] Decision: AVOID_PEER_ADJUSTMENT. Actor=ORCA:{self.robot_id}. '
                    f'Info: desired_vx={self.desired.linear.x:.2f} -> adjusted_vx={result.linear.x:.2f}, '
                    f'active_peers_tracked={len(peers)}.',
                    throttle_duration_sec=3.0
                )
        if not finite_command(result.linear.x, result.angular.z):
            result = Twist()
        self.pub.publish(result)


def main():
    rclpy.init(); node = OrcaNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

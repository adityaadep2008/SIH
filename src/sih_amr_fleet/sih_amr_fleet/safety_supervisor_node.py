import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sih_amr_interfaces.msg import RobotState, SafetyState
from std_msgs.msg import Bool, Float32

from .algorithms import braking_safe_speed, directional_scan_minimum, finite_command
from .common import FLEET_STATE_QOS, POSE_QOS, header, new_session_id, now_seconds


class SafetySupervisorNode(Node):
    """The final local authority before Gazebo/robot cmd_vel; all failures stop."""
    def __init__(self):
        super().__init__('safety_supervisor_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.deceleration = self.declare_parameter('braking_deceleration_mps2', 0.8).value
        self.margin = self.declare_parameter('braking_margin_m', 0.25).value
        # A 30-degree half-angle forward sector safely protects the travel direction
        # without side-shelf returns in 1.2m narrow aisles triggering false braking stops.
        self.forward_half_angle = self.declare_parameter('braking_sector_half_angle_rad', math.pi / 6.0).value
        self.lidar_yaw = self.declare_parameter('lidar_yaw_in_base_rad', math.pi / 2.0).value
        self.scan_timeout_s = self.declare_parameter('scan_timeout_s', 1.2).value
        self.localization_timeout_s = self.declare_parameter('localization_timeout_s', 1.2).value
        self.session_id, self.sequence, self.pose_time, self.scan_time = new_session_id(), 0, -math.inf, -math.inf
        self.nearest, self.candidate, self.estop = math.inf, Twist(), False
        self.measured_speed = 0.0
        self.scan_ranges, self.scan_angle_min, self.scan_angle_increment, self.scan_range_min = (), 0.0, 0.0, 0.0
        self._received_local_state = False
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', FLEET_STATE_QOS)
        self.state_pub = self.create_publisher(SafetyState, '/fleet/safety_state', FLEET_STATE_QOS)
        self.clear_pub = self.create_publisher(Bool, 'entrance_clear', FLEET_STATE_QOS)
        self.reverse_clearance_pub = self.create_publisher(Float32, 'reverse_clearance_m', POSE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(LaserScan, 'scan', self.on_scan, POSE_QOS)
        self.create_subscription(Twist, 'cmd_vel_candidate', lambda msg: setattr(self, 'candidate', msg), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'emergency_stop', lambda msg: setattr(self, 'estop', msg.data), FLEET_STATE_QOS)
        self.create_timer(0.1, self.enforce)

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose_time = now_seconds(self) if msg.localization_valid else -math.inf
            self.measured_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def on_local_state(self, msg):
        if msg.localization_valid:
            self.pose_time = now_seconds(self)
            self.measured_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
            if not self._received_local_state:
                self._received_local_state = True
                self.get_logger().info('Safety Supervisor received first local RobotState sample')

    def on_scan(self, scan):
        self.scan_ranges = tuple(scan.ranges)
        self.scan_angle_min = scan.angle_min
        self.scan_angle_increment = scan.angle_increment
        self.scan_range_min = scan.range_min
        self.scan_time = now_seconds(self)
        self.nearest = min((value for value in scan.ranges if math.isfinite(value) and value >= scan.range_min), default=math.inf)

    def directional_clearance(self, linear_x):
        if abs(linear_x) < 1e-4:
            return math.inf
        base_direction = 0.0 if linear_x > 0.0 else math.pi
        # directional_scan_minimum consumes angles in the scan frame.
        direction = base_direction - self.lidar_yaw
        return directional_scan_minimum(
            self.scan_ranges, self.scan_angle_min, self.scan_angle_increment,
            direction, self.forward_half_angle, self.scan_range_min)

    def enforce(self):
        age = now_seconds(self) - self.pose_time
        scan_age = now_seconds(self) - self.scan_time
        requested_speed = abs(self.candidate.linear.x)
        measured_braking = max(
            self.measured_speed * self.measured_speed /
            (2.0 * max(self.deceleration, 1e-6)) + self.margin,
            0.30)
        invalid_command = not finite_command(self.candidate.linear.x, self.candidate.angular.z)
        travel_clearance = self.directional_clearance(self.candidate.linear.x)
        stop = (self.estop or invalid_command or age > self.localization_timeout_s or scan_age > self.scan_timeout_s
                or travel_clearance <= measured_braking)
        safe_speed = braking_safe_speed(
            travel_clearance, self.deceleration, self.margin)
        speed_limited = not stop and requested_speed > safe_speed
        level = (SafetyState.STOP if stop else
                 (SafetyState.SLOW if speed_limited else SafetyState.CLEAR))
        reason = ('emergency stop' if self.estop else
                  ('nonfinite command' if invalid_command else
                   ('localization stale' if age > self.localization_timeout_s else
                    ('scan stale' if scan_age > self.scan_timeout_s else
                     ('braking envelope' if stop else
                      ('braking speed cap' if speed_limited else 'clear'))))))
        if stop:
            self.get_logger().warning(
                f'[{self.robot_id}:SafetySupervisor] Decision: SAFETY_STOP. Actor=SafetySupervisor:{self.robot_id}. '
                f'Reason={reason}. Info: clearance={travel_clearance:.2f}m, braking_req={measured_braking:.2f}m, '
                f'measured_speed={self.measured_speed:.2f}mps, candidate_vx={self.candidate.linear.x:.2f}mps, '
                f'nearest_obstacle={self.nearest:.2f}m.',
                throttle_duration_sec=3.0
            )
        elif speed_limited:
            self.get_logger().info(
                f'[{self.robot_id}:SafetySupervisor] Decision: SAFETY_SLOW. Actor=SafetySupervisor:{self.robot_id}. '
                f'Info: requested={requested_speed:.2f}mps capped to safe_speed={safe_speed:.2f}mps, '
                f'clearance={travel_clearance:.2f}m, braking_req={measured_braking:.2f}m.',
                throttle_duration_sec=4.0
            )

        cmd = Twist()
        if not stop:
            cmd.linear.x = math.copysign(
                min(requested_speed, safe_speed), self.candidate.linear.x)
            cmd.linear.y = self.candidate.linear.y
            cmd.linear.z = self.candidate.linear.z
            cmd.angular.x = self.candidate.angular.x
            cmd.angular.y = self.candidate.angular.y
            cmd.angular.z = self.candidate.angular.z
        self.cmd_pub.publish(cmd); self.sequence += 1
        state = SafetyState(); state.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.3)
        state.level, state.nearest_obstacle_m = level, self.nearest
        state.time_to_collision_s = self.nearest / max(requested_speed, 0.01)
        state.reason = reason
        self.state_pub.publish(state)
        self.clear_pub.publish(Bool(
            data=not stop and travel_clearance > measured_braking))
        self.reverse_clearance_pub.publish(Float32(data=self.directional_clearance(-1.0)))


def main():
    rclpy.init(); node = SafetySupervisorNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

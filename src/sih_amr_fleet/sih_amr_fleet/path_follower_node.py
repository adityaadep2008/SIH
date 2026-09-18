import math
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetEvent, RobotState, RoutePlan, SafetyState, TaskExecutionStatus
from std_msgs.msg import Bool, Float32

from .algorithms import reverse_recovery_allowed
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, clamp, header, new_session_id, now_seconds


class PathFollowerNode(Node):
    """Track rolling WHCA* routes without cutting turns or overshooting tasks."""
    def __init__(self):
        super().__init__('path_follower_node')
        self.max_speed = self.declare_parameter('max_speed_mps', 6.0).value
        self.tracking_speed = self.declare_parameter('path_tracking_speed_mps', 1.0).value
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.kp = self.declare_parameter('linear_kp', 0.8).value
        self.turn_in_place_threshold = self.declare_parameter('turn_in_place_threshold_rad', 0.40).value
        self.waypoint_tolerance = self.declare_parameter('waypoint_tolerance_m', 0.18).value
        self.task_stop_radius = self.declare_parameter('task_stop_radius_m', 0.40).value
        self.task_slow_radius = self.declare_parameter('task_slow_radius_m', 2.0).value
        self.recovery_reverse_m = self.declare_parameter('recovery_reverse_distance_m', 2.0).value
        self.recovery_speed_mps = self.declare_parameter('recovery_speed_mps', 0.20).value
        self.recovery_margin_m = self.declare_parameter('recovery_margin_m', 0.5).value
        self.recovery_trigger_s = self.declare_parameter('recovery_trigger_s', 1.0).value
        self.recovery_cooldown_s = self.declare_parameter('recovery_cooldown_s', 5.0).value
        self.final_docking_speed_mps = self.declare_parameter('final_docking_speed_mps', 0.15).value
        self.pose, self.route, self.clear, self.hold = None, None, True, False
        self.execution_target, self.execution_phase = None, None
        self.docking_target, self.docking_final, self.protected = None, False, False
        self.speed_cap, self.nearest, self.reverse_clearance = math.inf, math.inf, math.inf
        self.recovery_state, self.recovery_started = 'IDLE', 0.0
        self.recovery_start_pose = None
        self.recovery_cooldown_until = 0.0
        self.safety_stop_started = None
        self.last_progress_pose = None
        self.last_progress_time = 0.0
        self.session_id, self.sequence = new_session_id(), 0
        self._received_local_state = False
        self.pub = self.create_publisher(Twist, 'cmd_vel_desired', FLEET_STATE_QOS)
        self.event_pub = self.create_publisher(FleetEvent, '/fleet/recovery_event', PROTOCOL_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(Bool, 'corridor_motion_allowed', lambda msg: setattr(self, 'clear', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'corridor_protected', lambda msg: setattr(self, 'protected', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Float32, 'corridor_speed_cap', lambda msg: setattr(self, 'speed_cap', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Float32, 'nearest_obstacle_m', lambda msg: setattr(self, 'nearest', msg.data), POSE_QOS)
        self.create_subscription(Float32, 'reverse_clearance_m', lambda msg: setattr(self, 'reverse_clearance', msg.data), POSE_QOS)
        self.create_subscription(Pose2D, 'docking/target', lambda msg: setattr(self, 'docking_target', msg), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'docking/final_active', lambda msg: setattr(self, 'docking_final', msg.data), FLEET_STATE_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, 'task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_timer(0.1, self.control)

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose = msg.pose
    def on_local_state(self, msg):
        if msg.localization_valid:
            self.pose = msg.pose
            if not self._received_local_state:
                self._received_local_state = True
                self.get_logger().info('Path follower received first local RobotState sample')
    def on_route(self, msg): self.route = msg if msg.route_feasible else None
    def on_execution(self, msg):
        if msg.owner_robot_id == self.robot_id:
            if (msg.phase == TaskExecutionStatus.COMPLETED or
                    (self.route is not None and self.route.task_id != msg.task_id)):
                self.route = None
            self.execution_target = msg.target
            self.execution_phase = msg.phase
            self.hold = msg.phase in (TaskExecutionStatus.PICKUP_WAIT, TaskExecutionStatus.DROPOFF_WAIT, TaskExecutionStatus.COMPLETED)
    def on_safety(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            return
        now = now_seconds(self)
        if self.recovery_state == 'REVERSING' and msg.level == SafetyState.STOP:
            self.recovery_state = 'IDLE'
            self.recovery_cooldown_until = now + self.recovery_cooldown_s
            self.publish_event('recovery_blocked', f'retreat stopped by Safety Supervisor: {msg.reason}')
            return
        if msg.level != SafetyState.STOP or msg.reason != 'braking envelope':
            self.safety_stop_started = None
            return
        if self.safety_stop_started is None:
            self.safety_stop_started = now
        if (self.recovery_state == 'IDLE' and now >= self.recovery_cooldown_until
                and now - self.safety_stop_started >= self.recovery_trigger_s):
            self.recovery_state, self.recovery_started = 'VERIFY', now
            self.publish_event('recovery_stop', f'persistent safety trigger: {msg.reason}')
    def publish_event(self, event_type, detail):
        self.sequence += 1
        msg = FleetEvent(); msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 3.0)
        msg.severity, msg.event_type = 2, event_type
        pose = 'unknown' if self.pose is None else f'map=({self.pose.x:.2f},{self.pose.y:.2f})'
        msg.detail = (f'{detail}; {pose}; nearest={self.nearest:.2f}; '
                      f'rear_clearance={self.reverse_clearance:.2f}; protected={self.protected}')
        self.event_pub.publish(msg)

    def route_target(self):
        if self.pose is None or self.route is None or not self.route.waypoints:
            return None
        if len(self.route.waypoints) == 1:
            return self.route.waypoints[0]
        distances = [math.hypot(point.x - self.pose.x, point.y - self.pose.y)
                     for point in self.route.waypoints]
        nearest_index = min(range(len(distances)), key=distances.__getitem__)
        target_index = min(nearest_index + 1, len(distances) - 1)
        while target_index < len(distances) - 1 and distances[target_index] <= self.waypoint_tolerance:
            target_index += 1
        return self.route.waypoints[target_index]

    def steer_to(self, target, speed_limit, angular_limit=1.2):
        cmd = Twist()
        dx, dy = target.x - self.pose.x, target.y - self.pose.y
        desired = math.atan2(dy, dx)
        angular_error = math.atan2(
            math.sin(desired - self.pose.theta), math.cos(desired - self.pose.theta))
        cmd.angular.z = clamp(2.0 * angular_error, -angular_limit, angular_limit)
        # Differential-drive AMRs must finish a grid-direction change before
        # translating.  Moving while rotating from the south-facing docks was
        # the direct cause of robots 2 and 3 arcing into the south wall.
        if abs(angular_error) < self.turn_in_place_threshold:
            cmd.linear.x = max(0.0, speed_limit) * max(0.0, math.cos(angular_error))
        return cmd

    def task_speed_limit(self):
        limit = min(self.max_speed, self.tracking_speed, self.speed_cap)
        if (self.pose is None or self.execution_target is None or
                self.execution_phase not in (
                    TaskExecutionStatus.EN_ROUTE_PICKUP,
                    TaskExecutionStatus.EN_ROUTE_DROPOFF)):
            return limit, False
        distance = math.hypot(
            self.execution_target.x - self.pose.x,
            self.execution_target.y - self.pose.y)
        if distance <= self.task_stop_radius:
            return 0.0, True
        if distance < self.task_slow_radius:
            span = max(self.task_slow_radius - self.task_stop_radius, 1e-3)
            ratio = (distance - self.task_stop_radius) / span
            scaled = max(0.20, self.tracking_speed * ratio)
            limit = min(limit, scaled)
        return limit, False

    def control(self):
        cmd = Twist()
        now = now_seconds(self)
        if self.recovery_state == 'VERIFY':
            if now - self.recovery_started >= 0.5:
                if reverse_recovery_allowed(self.reverse_clearance, self.recovery_reverse_m, self.recovery_margin_m,
                                           self.protected, self.docking_final):
                    self.recovery_state, self.recovery_started = 'REVERSING', now
                    self.recovery_start_pose = None if self.pose is None else (self.pose.x, self.pose.y)
                    self.publish_event('recovery_retreat_started', f'reverse={self.recovery_reverse_m:.1f}m')
                else:
                    self.recovery_state = 'IDLE'
                    self.recovery_cooldown_until = now + self.recovery_cooldown_s
                    self.publish_event('recovery_blocked', 'unsafe reverse rejected')
        elif self.recovery_state == 'REVERSING':
            distance = 0.0 if self.pose is None or self.recovery_start_pose is None else math.hypot(
                self.pose.x - self.recovery_start_pose[0], self.pose.y - self.recovery_start_pose[1])
            timeout = 1.5 * self.recovery_reverse_m / max(self.recovery_speed_mps, 0.01) + 1.0
            if distance >= self.recovery_reverse_m:
                self.recovery_state = 'IDLE'
                self.recovery_cooldown_until = now + self.recovery_cooldown_s
                self.publish_event('recovery_retreat_complete', f'reversed={distance:.2f}m; requesting normal replanning')
            elif now - self.recovery_started >= timeout:
                self.recovery_state = 'IDLE'
                self.recovery_cooldown_until = now + self.recovery_cooldown_s
                self.publish_event('recovery_blocked', f'retreat timed out after {distance:.2f}m')
            else:
                cmd.linear.x = -self.recovery_speed_mps
        elif self.pose is not None and self.docking_final and self.docking_target and self.clear:
            cmd = self.steer_to(self.docking_target, self.final_docking_speed_mps, 0.8)
            self.get_logger().info(
                f'[{self.robot_id}:PathFollower] Decision: FINAL_DOCKING_STEER. Actor=PathFollower:{self.robot_id}. '
                f'Info: target=({self.docking_target.x:.2f}, {self.docking_target.y:.2f}), speed={self.final_docking_speed_mps:.2f}mps.',
                throttle_duration_sec=3.0
            )
        elif self.pose is not None and self.clear and not self.hold:
            speed_limit, at_task = self.task_speed_limit()
            target = self.route_target()
            if not at_task and target is not None:
                cmd = self.steer_to(target, speed_limit)
                dx, dy = target.x - self.pose.x, target.y - self.pose.y
                ang_err = math.atan2(math.sin(math.atan2(dy, dx) - self.pose.theta), math.cos(math.atan2(dy, dx) - self.pose.theta))
                if cmd.linear.x > 0.05:
                    if self.last_progress_pose is None:
                        self.last_progress_pose = (self.pose.x, self.pose.y)
                        self.last_progress_time = now
                    else:
                        d_prog = math.hypot(self.pose.x - self.last_progress_pose[0], self.pose.y - self.last_progress_pose[1])
                        if d_prog > 0.15:
                            self.last_progress_pose = (self.pose.x, self.pose.y)
                            self.last_progress_time = now
                        elif now - self.last_progress_time > 6.0:
                            if self.recovery_state == 'IDLE' and now >= self.recovery_cooldown_until:
                                self.recovery_state, self.recovery_started = 'VERIFY', now
                                self.publish_event('recovery_stop', 'persistent motion stall / peer block')
                                self.last_progress_time = now
                else:
                    self.last_progress_pose = None
                self.get_logger().info(
                    f'[{self.robot_id}:PathFollower] Decision: TRACK_WAYPOINT. Actor=PathFollower:{self.robot_id}. '
                    f'Info: pose=({self.pose.x:.2f}, {self.pose.y:.2f}), target=({target.x:.2f}, {target.y:.2f}), '
                    f'dist={math.hypot(dx, dy):.2f}m, ang_err={ang_err:.2f}rad, speed_limit={speed_limit:.2f}mps, '
                    f'cmd=(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}), corridor_clear={self.clear}.',
                    throttle_duration_sec=4.0
                )
            elif at_task:
                self.get_logger().info(
                    f'[{self.robot_id}:PathFollower] Decision: STOP_AT_TASK_RADIUS. Actor=PathFollower:{self.robot_id}. '
                    f'Info: within task stop radius ({self.task_stop_radius:.2f}m).',
                    throttle_duration_sec=4.0
                )
        elif self.pose is not None and not self.clear:
            self.get_logger().warning(
                f'[{self.robot_id}:PathFollower] Decision: HOLD_CORRIDOR_BLOCKED. Actor=PathFollower:{self.robot_id}. '
                f'Info: corridor_motion_allowed=False, speed_cap={self.speed_cap}.',
                throttle_duration_sec=4.0
            )
        elif self.hold:
            self.get_logger().info(
                f'[{self.robot_id}:PathFollower] Decision: HOLD_DWELLING. Actor=PathFollower:{self.robot_id}. '
                f'Info: phase={self.execution_phase} (dwelling at station).',
                throttle_duration_sec=5.0
            )
        self.pub.publish(cmd)


def main():
    rclpy.init(); node = PathFollowerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

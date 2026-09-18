import json
import math
import os
import pathlib
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from sih_amr_interfaces.msg import (
    BlockageObservation, CorridorProtocol, DockProtocol, FleetEvent, FleetHealth,
    RobotState, RoutePlan, SafetyState, TaskAnnouncement, TaskAssignment,
    TaskConsensus, TaskExecutionStatus, TrajectoryIntent
)

from .common import (
    FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, TASK_SOURCE_QOS,
    now_seconds, stamp_seconds, quaternion_to_euler, wrap_angle
)


ROSOUT_QOS = QoSProfile(
    depth=1000,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DataCollectionNode(Node):
    """Passive JSONL telemetry recorder for fleet benchmarking and offline analysis."""

    def __init__(self):
        super().__init__('data_collection_node')
        self.output_file = self.declare_parameter('output_file', '/tmp/sih_amr_fleet_telemetry.jsonl').value
        self.run_id = f'run_{int(time.time())}'
        self.file_handle = None
        self.raw_odom = {}
        self.seen_task_announcements = set()
        self.seen_task_receipts = set()
        self.last_assignments = {}
        self.last_execution_sequences = {}
        self.pipeline = {}
        self.map_origin_x = self.declare_parameter('map_origin_x', -22.5).value
        self.map_origin_y = self.declare_parameter('map_origin_y', -30.0).value
        self.map_resolution_m = self.declare_parameter('map_resolution_m', 0.5).value
        self.lean_telemetry = self.declare_parameter('lean_telemetry', True).value
        self.last_state_log_time = {}
        self.robot_ids = self.declare_parameter('robot_ids', [
            'robot_1', 'robot_2', 'robot_3', 'robot_4',
            'robot_5', 'robot_6', 'robot_7', 'robot_8'
        ]).value
        self.gazebo_ground_truth_poses = {}
        self.gz_node = None
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.pose_v_pb2 import Pose_V
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, '/world/default/pose/info', self.on_gz_poses)
            self.gz_node.subscribe(Pose_V, '/world/default/dynamic_pose/info', self.on_gz_poses)
            self.get_logger().info('DataCollectionNode subscribed to Gazebo ground truth pose stream')
        except Exception as exc:
            self.get_logger().warning(f'Could not subscribe to Gazebo ground truth transport: {exc}')

        try:
            path = pathlib.Path(self.output_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.file_handle = open(path, 'a', buffering=1, encoding='utf-8')
            git_rev = ''
            try:
                git_rev = subprocess.check_output(
                    ['git', 'rev-parse', 'HEAD'],
                    cwd=str(pathlib.Path(__file__).resolve().parent),
                    text=True,
                    stderr=subprocess.DEVNULL
                ).strip()
            except Exception:
                pass

            manifest = {
                'event_type': 'run_manifest',
                'run_id': self.run_id,
                'git_revision': git_rev,
                'world_layout_version': '1.2.0',
                'map_version': '0.3.0',
                'robot_count': len(self.robot_ids),
                'random_seed': int(os.environ.get('FLEET_RANDOM_SEED', '42')),
                'speed_limits': {
                    'tracking_speed_mps': float(os.environ.get('FLEET_TRACKING_SPEED_MPS', '0.46'))
                },
                'fault_profile_enabled': os.environ.get('FLEET_ENABLE_FAULTS', 'false') == 'true',
                'randomization_profile_enabled': os.environ.get('FLEET_RANDOM_TASKS', 'false') == 'true',
                'dds_domain_id': int(os.environ.get('ROS_DOMAIN_ID', '0')),
                'start_time_iso': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'start_epoch_s': time.time(),
                'schema_version': '0.6.0',
                'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
                'record_fields': ('logged_at is simulation time; wall_logged_at is host UTC epoch time; '
                                  'robot_state.map_pose is fleet map-relative state; robot_state.gazebo_odom '
                                  'is untransformed simulator wheel odometry; pipeline_diagnostic is a '
                                  'one-hertz correlated control-stage snapshot; ros_log records '
                                  'every warning, error, and fatal emitted through /rosout.'),
            }
            self.write_record(manifest)
            self.get_logger().info(f'DataCollectionNode logging to {self.output_file}')
            self.start_epoch = time.time()
            self.completed_tasks = set()
            self.safety_stops = 0
            self.blockage_count = 0
        except Exception as e:
            self.get_logger().error(f'Failed to open telemetry log {self.output_file}: {e}')

        # Subscriptions across fleet coordination topics
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_robot_state, FLEET_STATE_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_subscription(TrajectoryIntent, '/fleet/trajectory_intent', self.on_intent, PROTOCOL_QOS)
        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_corridor, PROTOCOL_QOS)
        self.create_subscription(TaskAnnouncement, '/fleet/task_announcement', self.on_task_announcement, TASK_SOURCE_QOS)
        self.create_subscription(String, '/fleet/task_wire', self.on_local_task_wire, FLEET_STATE_QOS)
        self.create_subscription(String, '/fleet/task_receipt', self.on_task_receipt, FLEET_STATE_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_consensus, PROTOCOL_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(BlockageObservation, '/fleet/blockage_observation', self.on_blockage, PROTOCOL_QOS)
        self.create_subscription(DockProtocol, '/fleet/dock_protocol', self.on_dock, PROTOCOL_QOS)
        self.create_subscription(FleetEvent, '/fleet/recovery_event', self.on_event, PROTOCOL_QOS)
        self.create_subscription(FleetEvent, '/fleet/collision_event', self.on_event, PROTOCOL_QOS)
        self.create_subscription(Log, '/rosout', self.on_ros_log, ROSOUT_QOS)
        for robot_id in self.robot_ids:
            self.pipeline[robot_id] = {}
            self.create_subscription(Odometry, f'/{robot_id}/odom',
                                     lambda msg, robot_id=robot_id: self.on_raw_odom(robot_id, msg), POSE_QOS)
            self.create_subscription(
                TaskAssignment, f'/{robot_id}/task_assignment', self.on_assignment,
                FLEET_STATE_QOS)
            self.create_subscription(
                TaskExecutionStatus, f'/{robot_id}/task_execution_status', self.on_execution,
                FLEET_STATE_QOS)
            self.create_subscription(
                RoutePlan, f'/{robot_id}/planned_route', self.on_route,
                FLEET_STATE_QOS)
            for topic, stage in (
                    ('cmd_vel_desired', 'desired_command'),
                    ('cmd_vel_candidate', 'orca_command'),
                    ('cmd_vel', 'final_command')):
                self.create_subscription(
                    Twist, f'/{robot_id}/{topic}',
                    lambda msg, robot_id=robot_id, stage=stage:
                    self.on_pipeline_twist(robot_id, stage, msg),
                    FLEET_STATE_QOS)
            for topic, stage in (
                    ('corridor_motion_allowed', 'corridor_allowed'),
                    ('corridor_protected', 'corridor_protected')):
                self.create_subscription(
                    Bool, f'/{robot_id}/{topic}',
                    lambda msg, robot_id=robot_id, stage=stage:
                    self.on_pipeline_bool(robot_id, stage, msg),
                    FLEET_STATE_QOS)
            for topic, stage, qos in (
                    ('corridor_speed_cap', 'corridor_speed_cap', FLEET_STATE_QOS),
                    ('nearest_obstacle_m', 'nearest_obstacle', POSE_QOS),
                    ('reverse_clearance_m', 'reverse_clearance', POSE_QOS)):
                self.create_subscription(
                    Float32, f'/{robot_id}/{topic}',
                    lambda msg, robot_id=robot_id, stage=stage:
                    self.on_pipeline_float(robot_id, stage, msg), qos)
        self.create_timer(1.0, self.publish_pipeline_diagnostics)

    def remember_pipeline(self, robot_id, stage, values):
        """Keep one JSON-safe latest sample for a diagnostic pipeline stage."""
        if robot_id not in self.pipeline:
            return
        sample = dict(values)
        sample['received_at'] = now_seconds(self)
        self.pipeline[robot_id][stage] = sample

    def on_pipeline_twist(self, robot_id, stage, msg):
        self.remember_pipeline(robot_id, stage, {
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'angular_z': float(msg.angular.z),
        })

    def on_pipeline_bool(self, robot_id, stage, msg):
        self.remember_pipeline(robot_id, stage, {'value': bool(msg.data)})

    def on_pipeline_float(self, robot_id, stage, msg):
        self.remember_pipeline(robot_id, stage, {'value': float(msg.data)})

    @staticmethod
    def diagnostic_value(sample):
        return None if sample is None else {
            key: value for key, value in sample.items() if key != 'received_at'
        }

    def publish_pipeline_diagnostics(self):
        """Explain the active gate from assignment through final actuation."""
        if self.lean_telemetry:
            return
        now = now_seconds(self)
        for robot_id in self.robot_ids:
            stages = self.pipeline.get(robot_id, {})
            state = stages.get('state')
            assignment = stages.get('assignment')
            execution = stages.get('execution')
            route = stages.get('route')
            desired = stages.get('desired_command')
            candidate = stages.get('orca_command')
            final = stages.get('final_command')
            corridor = stages.get('corridor_allowed')
            safety = stages.get('safety')

            target_distance = None
            speed = None
            if state is not None:
                speed = (state['vx'] ** 2 + state['vy'] ** 2) ** 0.5
            if state is not None and execution is not None:
                target_distance = (
                    (state['x'] - execution['target_x']) ** 2 +
                    (state['y'] - execution['target_y']) ** 2) ** 0.5

            route_age = None if route is None else now - route['received_at']
            blocker = 'IDLE_NO_ASSIGNMENT'
            if assignment is not None:
                blocker = 'WAITING_FOR_EXECUTOR_STATUS'
            if execution is not None:
                if execution['phase'] == TaskExecutionStatus.PICKUP_WAIT:
                    blocker = 'PICKUP_DWELL'
                elif execution['phase'] == TaskExecutionStatus.DROPOFF_WAIT:
                    blocker = 'DROPOFF_DWELL'
                elif execution['phase'] == TaskExecutionStatus.COMPLETED:
                    blocker = 'COMPLETED'
                elif route is None:
                    blocker = 'NO_ROUTE_RECEIVED'
                elif not route['route_feasible']:
                    blocker = 'ROUTE_INFEASIBLE'
                elif route['task_id'] != execution['task_id']:
                    blocker = 'ROUTE_TASK_MISMATCH'
                elif route_age is not None and route_age > 2.0:
                    blocker = 'ROUTE_STALE'
                elif corridor is not None and not corridor['value']:
                    blocker = 'CORRIDOR_MOTION_DENIED'
                elif target_distance is not None and target_distance <= 0.45:
                    blocker = ('ARRIVAL_SPEED_TOO_HIGH' if speed is not None and speed > 0.15
                               else 'ARRIVAL_WAITING_FOR_EXECUTOR_TICK')
                elif desired is None:
                    blocker = 'NO_PATH_FOLLOWER_COMMAND'
                elif abs(desired['linear_x']) < 0.01:
                    blocker = ('TURNING_IN_PLACE' if abs(desired['angular_z']) >= 0.01
                               else 'PATH_FOLLOWER_ZERO_COMMAND')
                elif (candidate is not None and
                      abs(candidate['linear_x']) < 0.25 * abs(desired['linear_x'])):
                    blocker = 'ORCA_LINEAR_VETO'
                elif (final is not None and
                      abs(final['linear_x']) < 0.25 * abs(candidate['linear_x'] if candidate else desired['linear_x'])):
                    blocker = 'SAFETY_LINEAR_VETO'
                elif (final is not None and abs(final['linear_x']) >= 0.05 and
                      speed is not None and speed < 0.02):
                    blocker = 'ACTUATION_NOT_FOLLOWING_COMMAND'
                else:
                    blocker = 'MOVING_TOWARD_TARGET'

            ages = {
                stage: float(now - sample['received_at'])
                for stage, sample in stages.items()
                if 'received_at' in sample
            }
            self.write_record({
                'event_type': 'pipeline_diagnostic',
                'robot_id': robot_id,
                'blocker': blocker,
                'target_distance_m': target_distance,
                'measured_speed_mps': speed,
                'stage_ages_s': ages,
                'state': self.diagnostic_value(state),
                'assignment': self.diagnostic_value(assignment),
                'execution': self.diagnostic_value(execution),
                'route': self.diagnostic_value(route),
                'desired_command': self.diagnostic_value(desired),
                'orca_command': self.diagnostic_value(candidate),
                'final_command': self.diagnostic_value(final),
                'corridor_allowed': self.diagnostic_value(corridor),
                'corridor_speed_cap': self.diagnostic_value(stages.get('corridor_speed_cap')),
                'corridor_protected': self.diagnostic_value(stages.get('corridor_protected')),
                'safety': self.diagnostic_value(safety),
                'nearest_obstacle': self.diagnostic_value(stages.get('nearest_obstacle')),
                'reverse_clearance': self.diagnostic_value(stages.get('reverse_clearance')),
            })

    def on_raw_odom(self, robot_id, msg):
        self.raw_odom[robot_id] = {
            'pose': {'x': float(msg.pose.pose.position.x), 'y': float(msg.pose.pose.position.y),
                     'z': float(msg.pose.pose.position.z), 'qx': float(msg.pose.pose.orientation.x),
                     'qy': float(msg.pose.pose.orientation.y), 'qz': float(msg.pose.pose.orientation.z),
                     'qw': float(msg.pose.pose.orientation.w)},
            'twist': {'vx': float(msg.twist.twist.linear.x), 'vy': float(msg.twist.twist.linear.y),
                      'vz': float(msg.twist.twist.linear.z), 'wz': float(msg.twist.twist.angular.z)},
            'stamp_s': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        }

    def on_gz_poses(self, msg):
        for p in msg.pose:
            name = p.name
            matched_robot = None
            for rid in self.robot_ids:
                if name in (rid, f"{rid}/turtlebot4"):
                    matched_robot = rid
                    break
            if matched_robot:
                roll, pitch, yaw = quaternion_to_euler(p.orientation)
                self.gazebo_ground_truth_poses[matched_robot] = {
                    'x': round(float(p.position.x), 4),
                    'y': round(float(p.position.y), 4),
                    'z': round(float(p.position.z), 4),
                    'roll': round(float(roll), 4),
                    'pitch': round(float(pitch), 4),
                    'yaw': round(float(yaw), 4),
                    'tilt': round(float(math.hypot(roll, pitch)), 4),
                }

    def compute_pose_error(self, map_pose, gz_pose):
        if gz_pose is None:
            return None
        dx = float(map_pose.x) - float(gz_pose['x'])
        dy = float(map_pose.y) - float(gz_pose['y'])
        dyaw = wrap_angle(float(map_pose.theta) - float(gz_pose['yaw']))
        return {
            'error_xy_m': round(math.hypot(dx, dy), 4),
            'error_x_m': round(dx, 4),
            'error_y_m': round(dy, 4),
            'error_yaw_rad': round(dyaw, 4),
            'is_tilted': bool(gz_pose.get('tilt', 0.0) > 0.15),
        }

    def write_record(self, record):
        if not self.file_handle:
            return
        try:
            record['logged_at'] = now_seconds(self)
            record['wall_logged_at'] = time.time()
            self.file_handle.write(json.dumps(record) + '\n')
        except Exception:
            pass  # Strictly passive; never fail or impede control

    def on_robot_state(self, msg):
        robot_id = msg.fleet_header.robot_id
        now = now_seconds(self)
        self.remember_pipeline(robot_id, 'state', {
            'x': float(msg.pose.x), 'y': float(msg.pose.y),
            'theta': float(msg.pose.theta),
            'vx': float(msg.twist.linear.x), 'vy': float(msg.twist.linear.y),
            'wz': float(msg.twist.angular.z),
            'localization_valid': bool(msg.localization_valid),
        })
        if self.lean_telemetry:
            last_t = self.last_state_log_time.get(robot_id, 0.0)
            if now - last_t < 1.0:
                return
            self.last_state_log_time[robot_id] = now
        gz_gt = self.gazebo_ground_truth_poses.get(robot_id)
        pose_err = self.compute_pose_error(msg.pose, gz_gt)
        self.write_record({
            'event_type': 'robot_state',
            'robot_id': robot_id,
            'session_id': msg.fleet_header.session_id,
            'seq': msg.fleet_header.sequence_no,
            'x': float(msg.pose.x),
            'y': float(msg.pose.y),
            'theta': float(msg.pose.theta),
            'vx': float(msg.twist.linear.x),
            'vy': float(msg.twist.linear.y),
            'wz': float(msg.twist.angular.z),
            'map_pose': {'x': float(msg.pose.x), 'y': float(msg.pose.y), 'theta': float(msg.pose.theta),
                         'cell': [round((msg.pose.x-self.map_origin_x)/self.map_resolution_m), round((msg.pose.y-self.map_origin_y)/self.map_resolution_m)]},
            'map_twist': {'vx': float(msg.twist.linear.x), 'vy': float(msg.twist.linear.y), 'wz': float(msg.twist.angular.z)},
            'gazebo_ground_truth': gz_gt,
            'pose_error': pose_err,
            'gazebo_odom': self.raw_odom.get(robot_id),
            'localization_valid': msg.localization_valid,
        })

    def on_health(self, msg):
        self.write_record({
            'event_type': 'health',
            'robot_id': msg.fleet_header.robot_id,
            'battery_percent': float(msg.battery_percent),
            'comm_state': int(msg.communication_state),
            'task_feasible': msg.task_feasible,
            'safety_ok': msg.safety_ok,
            'active_task_id': msg.active_task_id,
        })

    def on_safety(self, msg):
        self.remember_pipeline(msg.fleet_header.robot_id, 'safety', {
            'level': int(msg.level), 'reason': msg.reason,
            'nearest_obstacle_m': float(msg.nearest_obstacle_m),
            'ttc_s': float(msg.time_to_collision_s),
        })
        if self.lean_telemetry and int(msg.level) == 0:
            return  # Suppress logging 40 Hz normal safety states
        self.write_record({
            'event_type': 'safety_state',
            'robot_id': msg.fleet_header.robot_id,
            'level': int(msg.level),
            'nearest_obstacle_m': float(msg.nearest_obstacle_m),
            'ttc_s': float(msg.time_to_collision_s),
            'reason': msg.reason,
        })

    def on_intent(self, msg):
        self.write_record({
            'event_type': 'trajectory_intent',
            'robot_id': msg.fleet_header.robot_id,
            'plan_id': int(msg.plan_id),
            'cells_count': len(msg.reservations),
            'priority': int(msg.priority),
        })

    def on_corridor(self, msg):
        self.write_record({
            'event_type': 'corridor_protocol',
            'robot_id': msg.fleet_header.robot_id,
            'corridor_id': msg.corridor_id,
            'request_id': msg.request_id,
            'target_robot_id': msg.target_robot_id,
            'lamport_time': int(msg.lamport_time),
            'event': int(msg.event),
        })

    def on_consensus(self, msg):
        sig = (msg.fleet_header.robot_id, msg.task_id, msg.winner_robot_id, int(msg.assignment_epoch), int(msg.event))
        if self.lean_telemetry and sig in getattr(self, 'seen_consensus_sigs', set()):
            return
        if not hasattr(self, 'seen_consensus_sigs'):
            self.seen_consensus_sigs = set()
        self.seen_consensus_sigs.add(sig)
        self.write_record({
            'event_type': 'task_consensus',
            'robot_id': msg.fleet_header.robot_id,
            'robot_session_id': msg.fleet_header.session_id,
            'task_id': msg.task_id,
            'winner_robot_id': msg.winner_robot_id,
            'winner_session_id': msg.winner_session_id,
            'winning_bid': float(msg.winning_bid),
            'epoch': int(msg.assignment_epoch),
            'event': int(msg.event),
        })

    def on_assignment(self, msg):
        self.remember_pipeline(msg.owner_robot_id, 'assignment', {
            'task_id': msg.task.task_id,
            'owner_session_id': msg.owner_session_id,
            'epoch': int(msg.assignment_epoch),
            'active': bool(msg.active),
            'lease_until_s': stamp_seconds(msg.lease_until),
            'pickup_x': float(msg.task.pickup.x),
            'pickup_y': float(msg.task.pickup.y),
            'dropoff_x': float(msg.task.dropoff.x),
            'dropoff_y': float(msg.task.dropoff.y),
        })
        signature = (
            msg.owner_robot_id, msg.owner_session_id,
            int(msg.assignment_epoch), bool(msg.active))
        if self.last_assignments.get(msg.task.task_id) == signature:
            return
        self.last_assignments[msg.task.task_id] = signature
        self.write_record({
            'event_type': 'task_assignment',
            'robot_id': msg.owner_robot_id,
            'task_id': msg.task.task_id,
            'owner_session_id': msg.owner_session_id,
            'epoch': int(msg.assignment_epoch),
            'active': bool(msg.active),
        })

    def on_route(self, msg):
        robot_id = msg.fleet_header.robot_id
        self.remember_pipeline(robot_id, 'route', {
            'task_id': msg.task_id,
            'plan_id': int(msg.plan_id),
            'route_feasible': bool(msg.route_feasible),
            'failure_reason': msg.failure_reason,
            'valid_until_s': stamp_seconds(msg.fleet_header.valid_until),
            'cells': [
                [int(cell.x), int(cell.y), int(cell.time_slot)]
                for cell in msg.cells
            ],
            'waypoints': [
                [float(point.x), float(point.y), float(point.theta)]
                for point in msg.waypoints
            ],
        })
        self.write_record({
            'event_type': 'planned_route',
            'robot_id': robot_id,
            'task_id': msg.task_id,
            'plan_id': int(msg.plan_id),
            'route_feasible': bool(msg.route_feasible),
            'failure_reason': msg.failure_reason,
            'cells_count': len(msg.cells),
            'waypoints_count': len(msg.waypoints),
            'cells': [
                [int(cell.x), int(cell.y), int(cell.time_slot)]
                for cell in msg.cells
            ],
            'waypoints': [
                [float(point.x), float(point.y), float(point.theta)]
                for point in msg.waypoints
            ],
        })

    def on_task_announcement(self, msg):
        key = (msg.fleet_header.robot_id, msg.fleet_header.session_id, msg.fleet_header.sequence_no)
        if key in self.seen_task_announcements:
            return
        self.seen_task_announcements.add(key)
        self.write_record({
            'event_type': 'task_announcement',
            'task_id': msg.task.task_id,
            'source_robot_id': msg.fleet_header.robot_id,
            'source_session_id': msg.fleet_header.session_id,
            'source_seq': int(msg.fleet_header.sequence_no),
            'pickup_x': float(msg.task.pickup.x),
            'pickup_y': float(msg.task.pickup.y),
            'dropoff_x': float(msg.task.dropoff.x),
            'dropoff_y': float(msg.task.dropoff.y),
        })

    def on_local_task_wire(self, msg):
        """Record one workload event from the shared standard-message wire."""
        try:
            data = json.loads(msg.data)
            key = (data['source_robot_id'], data['source_session_id'], int(data['source_seq']))
            if key in self.seen_task_announcements:
                return
            self.seen_task_announcements.add(key)
            pickup, dropoff = data['pickup'], data['dropoff']
            self.write_record({
                'event_type': 'task_announcement', 'task_id': data['task_id'],
                'source_robot_id': data['source_robot_id'],
                'source_session_id': data['source_session_id'], 'source_seq': int(data['source_seq']),
                'pickup_x': float(pickup[0]), 'pickup_y': float(pickup[1]),
                'dropoff_x': float(dropoff[0]), 'dropoff_y': float(dropoff[1]),
                'transport': 'shared_standard_message_broadcast',
            })
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass  # Passive recorder: invalid observations cannot affect control.

    def on_task_receipt(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('event') != 'task_receipt':
                return
            key = (data.get('source_session_id', ''), int(data.get('source_seq', 0)),
                   data['task_id'], data['robot_id'])
            if key in self.seen_task_receipts:
                return
            self.seen_task_receipts.add(key)
            self.write_record({
                'event_type': 'task_receipt', 'task_id': data['task_id'],
                'robot_id': data['robot_id'], 'source_robot_id': data.get('source_robot_id', ''),
                'source_session_id': data.get('source_session_id', ''),
                'source_seq': int(data.get('source_seq', 0)),
            })
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def on_execution(self, msg):
        source = (msg.owner_robot_id, msg.fleet_header.session_id)
        sequence = int(msg.fleet_header.sequence_no)
        if sequence <= self.last_execution_sequences.get(source, -1):
            return
        self.last_execution_sequences[source] = sequence
        if int(msg.phase) == TaskExecutionStatus.COMPLETED and msg.task_id:
            self.completed_tasks.add(msg.task_id)
        self.remember_pipeline(msg.owner_robot_id, 'execution', {
            'task_id': msg.task_id,
            'session_id': msg.fleet_header.session_id,
            'sequence': sequence,
            'phase': int(msg.phase),
            'target_x': float(msg.target.x),
            'target_y': float(msg.target.y),
            'wait_remaining_s': float(msg.wait_time_remaining_s),
            'valid_until_s': stamp_seconds(msg.fleet_header.valid_until),
        })
        if self.lean_telemetry:
            if not hasattr(self, 'last_logged_phase'):
                self.last_logged_phase = {}
            if self.last_logged_phase.get((msg.owner_robot_id, msg.task_id)) == int(msg.phase):
                return
            self.last_logged_phase[(msg.owner_robot_id, msg.task_id)] = int(msg.phase)

        self.write_record({
            'event_type': 'task_execution',
            'task_id': msg.task_id,
            'robot_id': msg.owner_robot_id,
            'phase': int(msg.phase),
            'target_x': float(msg.target.x),
            'target_y': float(msg.target.y),
            'wait_remaining_s': float(msg.wait_time_remaining_s),
        })

    def on_blockage(self, msg):
        if self.lean_telemetry:
            if not hasattr(self, 'last_blockage_times'):
                self.last_blockage_times = {}
            now = now_seconds(self)
            if now - self.last_blockage_times.get(msg.fleet_header.robot_id, 0.0) < 1.0:
                return
            self.last_blockage_times[msg.fleet_header.robot_id] = now

        self.write_record({
            'event_type': 'blockage_observation',
            'robot_id': msg.fleet_header.robot_id,
            'confidence': float(msg.confidence),
            'source': msg.source,
            'cells_count': len(msg.cells),
            'cells': [[int(cell.x), int(cell.y)] for cell in msg.cells],
            'valid_until_s': stamp_seconds(msg.fleet_header.valid_until),
        })

    def on_dock(self, msg):
        self.write_record({'event_type': 'dock_protocol', 'robot_id': msg.fleet_header.robot_id,
                           'dock_id': msg.dock_id, 'request_id': msg.request_id,
                           'lamport_time': int(msg.lamport_time), 'event': int(msg.event)})

    def on_event(self, msg):
        self.write_record({'event_type': msg.event_type, 'robot_id': msg.fleet_header.robot_id,
                           'severity': int(msg.severity), 'detail': msg.detail,
                           'source': 'fleet_event'})

    def on_ros_log(self, msg):
        """Persist every warning/error from every ROS node in one JSONL stream."""
        if int(msg.level) < int(Log.WARN):
            return
        self.write_record({
            'event_type': 'ros_log',
            'severity': int(msg.level),
            'node_name': msg.name,
            'message': msg.msg,
            'source_file': msg.file,
            'source_function': msg.function,
            'source_line': int(msg.line),
            'ros_stamp_s': float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9,
        })

    def destroy_node(self):
        if self.file_handle:
            try:
                now_epoch = time.time()
                makespan = max(1.0, now_epoch - getattr(self, 'start_epoch', now_epoch))
                completed_cnt = len(getattr(self, 'completed_tasks', set()))
                throughput = (completed_cnt / makespan) * 3600.0
                summary = {
                    'event_type': 'run_summary',
                    'run_id': self.run_id,
                    'end_time_iso': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                    'end_epoch_s': now_epoch,
                    'makespan_s': round(makespan, 2),
                    'throughput_tasks_per_hour': round(throughput, 2),
                    'completed_task_count': completed_cnt,
                    'safety_stop_count': getattr(self, 'safety_stops', 0),
                    'active_blockage_count': getattr(self, 'blockage_count', 0),
                    'termination_reason': 'NORMAL_SHUTDOWN',
                }
                self.write_record(summary)
                self.file_handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Kinematic LiDAR Carrier Simulation Node for Decentralized Multi-AMR Fleet.

Replaces Gazebo wheel physics with a deterministic kinematic motion model:
1. Subscribes to final safety-gated velocity commands (/robot_N/cmd_vel).
2. Integrates kinematic equations at 50 Hz.
3. Enforces a swept circular footprint collision gate against warehouse shelves,
   walls, and peer AMRs.
4. Moves the lightweight Gazebo LiDAR carriers via Gazebo Sim's /set_pose_vector.
5. Generates simulated wheel odometry (/robot_N/odom) with configurable noise.
6. Simulates independent physical anchor detection (dock contact confirmation).
7. Publishes hidden true_pose solely for benchmark validation and telemetry.
"""

import concurrent.futures
import math
import os
import pathlib
import random
import time
import yaml
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sih_amr_interfaces.msg import DockProtocol, FleetHeader, RobotState

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from .algorithms import body_velocity_to_map
from .common import (
    FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds,
    quaternion_to_euler, wrap_angle
)
from .map_geometry import map_geometry_from_data

ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)

try:
    import gz.transport13
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    GZ_TRANSPORT_AVAILABLE = True
except Exception:
    GZ_TRANSPORT_AVAILABLE = False


class RobotKinematicState:
    """Internal hidden simulation state for one kinematic AMR."""
    def __init__(self, robot_id: str, x: float, y: float, yaw: float):
        self.robot_id = robot_id
        # Hidden true world pose
        self.true_x = x
        self.true_y = y
        self.true_yaw = yaw
        self.true_vx = 0.0
        self.true_vy = 0.0
        self.true_wz = 0.0
        
        # Latest received commanded velocities (post-safety)
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = -math.inf
        
        # Simulated local wheel odometry integration
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0
        self.odom_seq = 0
        
        # Collision / stall state
        self.stalled = False
        
        # Dock confirmation state
        self.dock_confirmed = False
        self.current_dock_id = None

        # Gazebo pose observation and divergence tracking
        self.gz_observed_x = x
        self.gz_observed_y = y
        self.gz_observed_yaw = yaw
        self.last_gz_observed_time = -math.inf
        self.divergence_hold = False
        self.consecutive_healthy_observations = 0
        self.consecutive_divergent_observations = 0


class KinematicCarrierNode(Node):
    """Centralized simulation-side carrier driver and sensor synthesizer."""

    def __init__(self):
        super().__init__('kinematic_carrier_node')
        
        self.world_name = self.declare_parameter('world', 'default').value
        self.map_file = self.declare_parameter('map_file', '').value
        self.robot_radius = self.declare_parameter('robot_radius_m', 0.17).value
        self.update_rate_hz = self.declare_parameter('update_rate_hz', 50.0).value
        self.gz_request_timeout_ms = self.declare_parameter('gz_request_timeout_ms', 250).value
        self.gz_dispatch_rate_hz = self.declare_parameter('gz_dispatch_rate_hz', 20.0).value
        self.divergence_threshold_m = self.declare_parameter('divergence_threshold_m', 0.50).value
        self.divergence_consecutive_trigger = self.declare_parameter('divergence_consecutive_trigger', 3).value
        
        # Odometry noise parameters - default 0.0 per user instruction
        self.odom_scale_error = self.declare_parameter('odom_scale_error', 0.0).value
        self.gyro_drift_rate = self.declare_parameter('gyro_drift_rate_rps', 0.0).value
        self.odom_noise_std = self.declare_parameter('odom_noise_std_m', 0.0).value
        self.yaw_noise_std = self.declare_parameter('yaw_noise_std_rad', 0.0).value
        
        self.session_id = new_session_id()
        self.telemetry_sequence = 0
        self.gz_sync_total_count = 0
        self.gz_sync_success_count = 0
        self.gz_sync_timeout_count = 0
        self.gz_sync_rejected_count = 0
        self.gz_sync_exception_count = 0
        self.gz_sync_error_count = 0
        self.gz_coalesced_count = 0
        self.last_gz_latency_ms = 0.0
        self.max_gz_latency_ms = 0.0
        self.gz_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.gz_future = None
        self.latest_gz_pose_vector = None
        
        # Load map geometry for swept footprint collision checking
        self.map_resolution = 0.5
        self.map_width = 90
        self.map_height = 120
        self.map_origin_x = -22.5
        self.map_origin_y = -30.0
        self.blocked_cells: Set[Tuple[int, int]] = set()
        self.dock_anchors: Dict[str, Tuple[float, float, float]] = {}
        self.physical_shelves: List[Tuple[float, float, float, float]] = []
        
        self._load_map()
        
        # Determine active robots and initial poses
        fleet_count = int(os.environ.get('FLEET_COUNT', '4'))
        self.robots: Dict[str, RobotKinematicState] = {}
        self.odom_publishers: Dict[str, any] = {}
        
        for i in range(1, fleet_count + 1):
            rid = f'robot_{i}'
            x_val = os.environ.get(f'ROBOT_{i}_X')
            y_val = os.environ.get(f'ROBOT_{i}_Y')
            yaw_val = os.environ.get(f'ROBOT_{i}_YAW')
            
            if x_val is not None and y_val is not None and yaw_val is not None:
                init_x = float(x_val)
                init_y = float(y_val)
                init_yaw = float(yaw_val)
            else:
                dock_key = f'charging_pad_{i}'
                if dock_key in self.dock_anchors:
                    init_x, init_y, _ = self.dock_anchors[dock_key]
                    init_yaw = 1.5708  # Default facing north into warehouse aisle
                else:
                    init_x, init_y, init_yaw = 0.0, 0.0, 1.5708
                
            self.robots[rid] = RobotKinematicState(rid, init_x, init_y, init_yaw)

            
            # Subscribe to robot's safety-approved cmd_vel
            self.create_subscription(
                Twist, f'/{rid}/cmd_vel',
                lambda msg, r=rid: self._on_cmd_vel(r, msg),
                FLEET_STATE_QOS
            )
            
            # Publisher for simulated wheel odometry
            self.odom_publishers[rid] = self.create_publisher(
                Odometry, f'/{rid}/odom', ODOM_QOS
            )
            
        # Global publishers
        self.dock_protocol_pub = self.create_publisher(
            DockProtocol, '/fleet/dock_protocol', PROTOCOL_QOS
        )
        self.true_pose_pub = self.create_publisher(
            RobotState, '/simulation/true_pose', FLEET_STATE_QOS
        )
        
        # Gazebo Sim Transport client for batch pose updates
        self.gz_node = None
        self.gz_entity_ids: Dict[str, int] = {}
        if GZ_TRANSPORT_AVAILABLE:
            try:
                self.gz_node = gz.transport13.Node()
                self.gz_node.subscribe(
                    GzPose_V, f'/world/{self.world_name}/pose/info', self._on_gz_poses
                )
                self.gz_node.subscribe(
                    GzPose_V, f'/world/{self.world_name}/dynamic_pose/info', self._on_gz_poses
                )
                self.get_logger().info('Initialized Gazebo Sim Transport for carrier pose dispatcher and pose discovery')
            except Exception as e:
                self.get_logger().warning(f'Could not initialize gz.transport13.Node: {e}')
        else:
            self.get_logger().warning('gz.transport13 is not available; carrier poses will not sync to Gazebo')

        # Simulation clock handling
        self.last_sim_time = None
        self.create_subscription(Clock, '/clock', self._on_clock, POSE_QOS)
        
        # 50 Hz simulation tick timer
        timer_period = 1.0 / max(1.0, self.update_rate_hz)
        self.create_timer(timer_period, self._simulation_step)

        # Decoupled 20 Hz Gazebo batch pose dispatch timer
        gz_timer_period = 1.0 / max(1.0, float(self.gz_dispatch_rate_hz))
        self.create_timer(gz_timer_period, self._gz_dispatch_step)
        
        self.get_logger().info(
            f'KinematicCarrierNode active for {len(self.robots)} AMRs at {self.update_rate_hz} Hz '
            f'(Gazebo sync at {self.gz_dispatch_rate_hz} Hz, timeout={self.gz_request_timeout_ms} ms). '
            f'Odom noise={self.odom_noise_std}, scale_error={self.odom_scale_error}, gyro_drift={self.gyro_drift_rate}'
        )

    def _load_map(self):
        map_path = self.map_file
        if not map_path:
            # Try default share path
            from ament_index_python.packages import get_package_share_directory
            try:
                map_path = os.path.join(get_package_share_directory('sih_amr_fleet'), 'maps', 'demo_warehouse.yaml')
            except Exception:
                pass

        if map_path and os.path.exists(map_path):
            try:
                raw_data = yaml.safe_load(pathlib.Path(map_path).read_text()) or {}
                (self.map_resolution, self.map_width, self.map_height,
                 self.map_origin_x, self.map_origin_y, self.blocked_cells) = map_geometry_from_data(raw_data)
                
                for dock_id, spec in raw_data.get('anchors', {}).items():
                    pose = spec.get('map_pose', [])
                    if len(pose) >= 3:
                        self.dock_anchors[dock_id] = (float(pose[0]), float(pose[1]), float(pose[2]))
                self.get_logger().info(
                    f'Loaded map geometry: {len(self.blocked_cells)} blocked cells, '
                    f'{len(self.dock_anchors)} dock anchors from {map_path}'
                )
            except Exception as e:
                self.get_logger().error(f'Error reading map file {map_path}: {e}')

        # Load physical shelf bounding boxes from warehouse_layout.lock.yaml if available
        layout_candidates = [
            os.environ.get('WAREHOUSE_LAYOUT_FILE', ''),
            os.path.join(os.getcwd(), 'warehouse_layout.lock.yaml'),
            '/home/rtsws/amr_ws/src/SIH/warehouse_layout.lock.yaml',
            '/home/rtsws/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_layout.lock.yaml',
        ]
        layout_path = None
        for candidate in layout_candidates:
            if candidate and os.path.exists(candidate):
                layout_path = candidate
                break

        if layout_path:
            try:
                layout_data = yaml.safe_load(pathlib.Path(layout_path).read_text()) or {}
                for obj in layout_data.get('objects', []):
                    if obj.get('type') == 'storage_shelf':
                        bb = obj.get('bounding_box_2d_m')
                        if bb:
                            self.physical_shelves.append(
                                (float(bb['min_x']), float(bb['min_y']), float(bb['max_x']), float(bb['max_y']))
                            )
                self.get_logger().info(
                    f'Loaded {len(self.physical_shelves)} physical shelf bounding boxes from {layout_path}'
                )
            except Exception as e:
                self.get_logger().error(f'Error reading warehouse layout file {layout_path}: {e}')

    def _on_cmd_vel(self, robot_id: str, msg: Twist):
        robot = self.robots.get(robot_id)
        if robot:
            robot.cmd_linear_x = msg.linear.x
            robot.cmd_angular_z = msg.angular.z
            robot.last_cmd_time = now_seconds(self)

    def _on_clock(self, msg: Clock):
        sim_s = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.last_sim_time = sim_s

    def _on_gz_poses(self, msg: GzPose_V):
        """Cache Gazebo Sim entity IDs and monitor map<->Gazebo spatial divergence."""
        now = time.time()
        for p in msg.pose:
            for rid, robot in self.robots.items():
                if p.name in (f'{rid}/turtlebot4', rid):
                    if p.id != 0 and self.gz_entity_ids.get(rid) != p.id:
                        self.gz_entity_ids[rid] = p.id
                        self.get_logger().info(f'Resolved Gazebo entity ID for {rid} ({p.name}) -> {p.id}')

                    robot.gz_observed_x = float(p.position.x)
                    robot.gz_observed_y = float(p.position.y)
                    _, _, yaw = quaternion_to_euler(p.orientation)
                    robot.gz_observed_yaw = float(yaw)
                    robot.last_gz_observed_time = now

                    err_dist = math.hypot(robot.true_x - robot.gz_observed_x, robot.true_y - robot.gz_observed_y)
                    if err_dist > self.divergence_threshold_m:
                        robot.consecutive_divergent_observations += 1
                        robot.consecutive_healthy_observations = 0
                        if robot.consecutive_divergent_observations >= self.divergence_consecutive_trigger:
                            if not robot.divergence_hold:
                                robot.divergence_hold = True
                                self.get_logger().warning(
                                    f'[Carrier] AMR {rid} held due to persistent divergence: '
                                    f'error={err_dist:.3f}m > {self.divergence_threshold_m:.2f}m for {robot.consecutive_divergent_observations} frames '
                                    f'(true=({robot.true_x:.3f}, {robot.true_y:.3f}), gz=({robot.gz_observed_x:.3f}, {robot.gz_observed_y:.3f}))',
                                    throttle_duration_sec=2.0
                                )
                    else:
                        robot.consecutive_divergent_observations = 0
                        if err_dist <= 0.05:
                            robot.consecutive_healthy_observations += 1
                            if robot.divergence_hold and robot.consecutive_healthy_observations >= 5:
                                robot.divergence_hold = False
                                self.get_logger().info(
                                    f'[Carrier] AMR {rid} divergence resolved (consecutive healthy observations >= 5). '
                                    f'Resuming normal motion.'
                                )

    def _is_position_collision_free(self, robot_id: str, x: float, y: float, curr_x: Optional[float] = None, curr_y: Optional[float] = None) -> bool:
        """Check if a circular footprint at (x, y) intersects shelves, walls, or other AMRs.
        Allows inward escape if robot is already slightly outside the warehouse perimeter."""
        # Warehouse boundary limits with robot radius margin.
        # Perimeter transit lane centers are at x = -22.5 and x = +22.5.
        # Allow +/- 0.05m clearance so AMRs centered at -22.5 or +22.5 are not clamped.
        min_x = self.map_origin_x - 0.05
        max_x = self.map_origin_x + (self.map_width * self.map_resolution) + 0.05
        min_y = self.map_origin_y + 0.15
        max_y = self.map_origin_y + (self.map_height * self.map_resolution) - 0.15

        in_bounds = (min_x <= x <= max_x and min_y <= y <= max_y)
        if not in_bounds:
            if curr_x is not None and curr_y is not None:
                curr_viol = max(0.0, min_x - curr_x, curr_x - max_x, min_y - curr_y, curr_y - max_y)
                cand_viol = max(0.0, min_x - x, x - max_x, min_y - y, y - max_y)
                # Permit candidate position only if it strictly reduces current boundary violation
                if cand_viol >= curr_viol:
                    return False
            else:
                return False

        if self.physical_shelves:
            # Physical shelf obstacle check: exact distance from circular footprint to physical shelf boxes
            for min_bx, min_by, max_bx, max_by in self.physical_shelves:
                # Fast AABB reject before distance calculation
                if x < min_bx - self.robot_radius or x > max_bx + self.robot_radius:
                    continue
                if y < min_by - self.robot_radius or y > max_by + self.robot_radius:
                    continue
                dx_box = max(0.0, max(min_bx - x, x - max_bx))
                dy_box = max(0.0, max(min_by - y, y - max_by))
                if math.hypot(dx_box, dy_box) < self.robot_radius:
                    return False
        else:
            # Fallback: Static map obstacle check: exact point-to-box Euclidean distance
            half_res = 0.5 * self.map_resolution
            cell_radius = math.ceil(self.robot_radius / self.map_resolution) + 1
            center_cx = round((x - self.map_origin_x) / self.map_resolution)
            center_cy = round((y - self.map_origin_y) / self.map_resolution)

            for dcx in range(-cell_radius, cell_radius + 1):
                for dcy in range(-cell_radius, cell_radius + 1):
                    cell = (center_cx + dcx, center_cy + dcy)
                    if cell in self.blocked_cells:
                        cell_x = self.map_origin_x + cell[0] * self.map_resolution
                        cell_y = self.map_origin_y + cell[1] * self.map_resolution
                        # Distance from robot circle center to axis-aligned obstacle box
                        dx_box = max(0.0, abs(x - cell_x) - half_res)
                        dy_box = max(0.0, abs(y - cell_y) - half_res)
                        if math.hypot(dx_box, dy_box) < self.robot_radius:
                            return False

        # Inter-AMR collision check
        for other_id, other_robot in self.robots.items():
            if other_id != robot_id:
                dist = math.hypot(x - other_robot.true_x, y - other_robot.true_y)
                if dist < (2.0 * self.robot_radius):
                    return False

        return True

    def _simulation_step(self):
        """Execute one simulation tick: update AMR positions, collisions, odometry, and queue Gazebo pose update."""
        now_s = now_seconds(self)
        now_wall = time.time()
        dt = 1.0 / max(1.0, self.update_rate_hz)
        
        gz_pose_vector = GzPose_V() if (GZ_TRANSPORT_AVAILABLE and self.gz_node) else None

        for robot_id, robot in self.robots.items():
            # Check stale Gazebo pose feedback while in motion (wall time)
            if robot.last_gz_observed_time > 0 and (now_wall - robot.last_gz_observed_time) > 3.0:
                if abs(robot.cmd_linear_x) > 0.01 or abs(robot.cmd_angular_z) > 0.01:
                    if not robot.divergence_hold:
                        robot.divergence_hold = True
                        self.get_logger().warning(
                            f'[Carrier] AMR {robot_id} held due to stale Gazebo pose stream '
                            f'({now_wall - robot.last_gz_observed_time:.2f}s > 3.0s)',
                            throttle_duration_sec=2.0
                        )

            # Evaluate commanded velocity
            if now_s - robot.last_cmd_time > 0.5:
                # Timeout commanded velocity
                v = 0.0
                w = 0.0
            else:
                v = robot.cmd_linear_x
                w = robot.cmd_angular_z

            # Kinematic candidate step
            dx = v * math.cos(robot.true_yaw) * dt
            dy = v * math.sin(robot.true_yaw) * dt
            dyaw = w * dt

            target_x = robot.true_x + dx
            target_y = robot.true_y + dy
            target_yaw = wrap_angle(robot.true_yaw + dyaw)

            # Swept-footprint check (subdivide movement to prevent tunnelling)
            step_dist = math.hypot(dx, dy)
            substeps = max(1, math.ceil(step_dist / 0.02))
            allowed_ratio = 1.0
            collision = False

            if step_dist > 1e-4:
                for s in range(1, substeps + 1):
                    fraction = s / substeps
                    cx = robot.true_x + fraction * dx
                    cy = robot.true_y + fraction * dy
                    if not self._is_position_collision_free(robot_id, cx, cy, curr_x=robot.true_x, curr_y=robot.true_y):
                        collision = True
                        allowed_ratio = max(0.0, (s - 1) / substeps)
                        break

            if collision:
                robot.stalled = True
                actual_dx = allowed_ratio * dx
                actual_dy = allowed_ratio * dy
                # Unconditionally allow in-place rotation so circular AMR can pivot away
                actual_dyaw = dyaw
            else:
                robot.stalled = False
                actual_dx = dx
                actual_dy = dy
                actual_dyaw = dyaw

            # Update hidden true pose
            robot.true_x += actual_dx
            robot.true_y += actual_dy
            robot.true_yaw = wrap_angle(robot.true_yaw + actual_dyaw)
            robot.true_vx = (actual_dx / dt) if dt > 0 else 0.0
            robot.true_vy = (actual_dy / dt) if dt > 0 else 0.0
            robot.true_wz = (actual_dyaw / dt) if dt > 0 else 0.0

            # Append to Gazebo pose vector message (only for entities with resolved IDs)
            if gz_pose_vector is not None and robot_id in self.gz_entity_ids:
                p = gz_pose_vector.pose.add()
                p.id = self.gz_entity_ids[robot_id]
                p.name = f'{robot_id}/turtlebot4'
                p.position.x = robot.true_x
                p.position.y = robot.true_y
                p.position.z = 0.03
                # Convert planar yaw to quaternion
                half_yaw = robot.true_yaw * 0.5
                p.orientation.x = 0.0
                p.orientation.y = 0.0
                p.orientation.z = math.sin(half_yaw)
                p.orientation.w = math.cos(half_yaw)

            # Generate simulated wheel odometry
            true_dist = math.copysign(math.hypot(actual_dx, actual_dy), v)
            scale = 1.0 + self.odom_scale_error
            noise_dist = random.gauss(0.0, self.odom_noise_std) if self.odom_noise_std > 0 else 0.0
            noise_yaw = random.gauss(0.0, self.yaw_noise_std) if self.yaw_noise_std > 0 else 0.0
            
            meas_dist = true_dist * scale + noise_dist
            meas_dyaw = actual_dyaw + (self.gyro_drift_rate * dt) + noise_yaw

            robot.local_x += meas_dist * math.cos(robot.local_yaw)
            robot.local_y += meas_dist * math.sin(robot.local_yaw)
            robot.local_yaw = wrap_angle(robot.local_yaw + meas_dyaw)
            robot.odom_seq += 1

            self._publish_odometry(robot, dt, v, w)
            self._check_anchor_proximity(robot)

        # Buffer latest absolute poses for decoupled Gazebo dispatch
        if gz_pose_vector is not None and len(gz_pose_vector.pose) > 0:
            self.latest_gz_pose_vector = gz_pose_vector

        # Publish true pose telemetry for validation
        self._publish_telemetry_true_poses()

    def _gz_dispatch_step(self):
        """Decoupled Gazebo batch pose dispatch running at 20 Hz with latest-only coalescing."""
        if not GZ_TRANSPORT_AVAILABLE or self.gz_node is None or self.gz_executor is None:
            return
        if self.latest_gz_pose_vector is None or len(self.latest_gz_pose_vector.pose) == 0:
            return

        if self.gz_future is not None and not self.gz_future.done():
            self.gz_coalesced_count += 1
            return

        batch = self.latest_gz_pose_vector
        self.latest_gz_pose_vector = None
        self.gz_future = self.gz_executor.submit(self._dispatch_gz_pose, batch)

    def _dispatch_gz_pose(self, gz_pose_vector):
        """Execute Gazebo set_pose_vector request in background worker to prevent choking the motion timer."""
        t0 = time.perf_counter()
        self.gz_sync_total_count += 1
        try:
            res, rep = self.gz_node.request(
                f'/world/{self.world_name}/set_pose_vector',
                gz_pose_vector,
                GzPose_V,
                GzBoolean,
                timeout=int(self.gz_request_timeout_ms)
            )
            lat_ms = (time.perf_counter() - t0) * 1000.0
            self.last_gz_latency_ms = lat_ms
            if lat_ms > self.max_gz_latency_ms:
                self.max_gz_latency_ms = lat_ms

            if res and rep is not None and rep.data:
                self.gz_sync_success_count += 1
            elif res and rep is not None and not rep.data:
                self.gz_sync_rejected_count += 1
                self.gz_sync_error_count += 1
                self.get_logger().warning(
                    f'[Carrier] Gazebo set_pose_vector rejected by server. '
                    f'Rejections: {self.gz_sync_rejected_count}, Total errors: {self.gz_sync_error_count}/{self.gz_sync_total_count}.',
                    throttle_duration_sec=5.0
                )
            else:
                self.gz_sync_timeout_count += 1
                self.gz_sync_error_count += 1
                self.get_logger().warning(
                    f'[Carrier] Gazebo set_pose_vector deadline miss / timeout ({lat_ms:.1f}ms > {self.gz_request_timeout_ms}ms). '
                    f'Timeouts: {self.gz_sync_timeout_count}, Total errors: {self.gz_sync_error_count}/{self.gz_sync_total_count}.',
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            lat_ms = (time.perf_counter() - t0) * 1000.0
            self.last_gz_latency_ms = lat_ms
            self.gz_sync_exception_count += 1
            self.gz_sync_error_count += 1
            self.get_logger().warning(
                f'[Carrier] Exception in Gazebo set_pose_vector: {e}. '
                f'Exceptions: {self.gz_sync_exception_count}, Total errors: {self.gz_sync_error_count}/{self.gz_sync_total_count}.',
                throttle_duration_sec=5.0
            )

    def _publish_odometry(self, robot: RobotKinematicState, dt: float, cmd_v: float, cmd_w: float):
        """Publish standard nav_msgs/Odometry for localization_node."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = robot.local_x
        odom_msg.pose.pose.position.y = robot.local_y
        odom_msg.pose.pose.position.z = 0.0
        half_yaw = robot.local_yaw * 0.5
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw)
        
        # Position covariance
        cov_val = 0.001 if (self.odom_noise_std == 0.0 and self.odom_scale_error == 0.0) else 0.02
        odom_msg.pose.covariance[0] = cov_val
        odom_msg.pose.covariance[7] = cov_val
        odom_msg.pose.covariance[35] = cov_val
        
        # Body-frame twist reflects actual chassis motion
        odom_msg.twist.twist.linear.x = robot.true_vx
        odom_msg.twist.twist.angular.z = robot.true_wz
        
        self.odom_publishers[robot.robot_id].publish(odom_msg)

    def _check_anchor_proximity(self, robot: RobotKinematicState):
        """Simulate independent physical contact or beacon confirmation at a dock anchor."""
        dock_tolerance_dist = 0.08  # 8 cm contact tolerance
        dock_tolerance_yaw = 0.20   # 0.20 rad heading tolerance

        matched_dock = None
        for dock_id, anchor_pose in self.dock_anchors.items():
            dx = robot.true_x - anchor_pose[0]
            dy = robot.true_y - anchor_pose[1]
            dist = math.hypot(dx, dy)
            dyaw = abs(wrap_angle(robot.true_yaw - anchor_pose[2]))
            
            if dist <= dock_tolerance_dist and dyaw <= dock_tolerance_yaw:
                matched_dock = dock_id
                break


        if matched_dock:
            # Trigger confirmation once when entering dock zone and stopped
            if not robot.dock_confirmed and abs(robot.true_vx) < 0.05 and abs(robot.true_vy) < 0.05:
                robot.dock_confirmed = True
                robot.current_dock_id = matched_dock
                
                dock_msg = DockProtocol()
                dock_msg.fleet_header = header(
                    self, robot.robot_id, self.session_id, robot.odom_seq, 0.5
                )
                dock_msg.dock_id = matched_dock
                dock_msg.event = DockProtocol.CONFIRMED
                self.dock_protocol_pub.publish(dock_msg)
                self.get_logger().info(
                    f'Simulated physical anchor confirmation: {robot.robot_id} at {matched_dock} '
                    f'(true_x={robot.true_x:.3f}, true_y={robot.true_y:.3f})'
                )
        else:
            # Reset dock state when AMR departs from dock
            if robot.dock_confirmed:
                robot.dock_confirmed = False
                robot.current_dock_id = None

    def _publish_telemetry_true_poses(self):
        """Publish hidden true poses to /simulation/true_pose for telemetry only."""
        now = self.get_clock().now()
        for robot_id, robot in self.robots.items():
            self.telemetry_sequence += 1
            msg = RobotState()
            msg.fleet_header = header(self, robot_id, self.session_id, self.telemetry_sequence, 0.2)
            msg.pose.x = robot.true_x
            msg.pose.y = robot.true_y
            msg.pose.theta = robot.true_yaw
            msg.twist.linear.x = robot.true_vx
            msg.twist.linear.y = robot.true_vy
            msg.twist.angular.z = robot.true_wz
            msg.localization_valid = True
            self.true_pose_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'gz_executor') and self.gz_executor:
            self.gz_executor.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicCarrierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

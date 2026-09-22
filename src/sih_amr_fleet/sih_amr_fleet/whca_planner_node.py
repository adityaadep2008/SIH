import math
import pathlib
import yaml
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sih_amr_interfaces.msg import (
    BlockageObservation, CorridorProtocol, GridCell, RobotState, RoutePlan, TaskAssignment,
    TaskExecutionStatus, TrajectoryIntent
)

from .algorithms import (
    clear_nearfield_blockages, lane_waypoint_overrides, whca_star
)
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds
from .map_geometry import map_geometry_from_data
from .warehouse_tasks import narrow_lanes


class WhcaPlannerNode(Node):
    """Rolling cooperative A* planner over a configured warehouse occupancy grid."""

    def __init__(self):
        super().__init__('whca_planner_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        map_file = self.declare_parameter('map_file', '').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        self.horizon = self.declare_parameter('horizon_steps', 12).value
        self.reservation_buffer_cells = self.declare_parameter('reservation_buffer_cells', 3).value

        self.session_id = new_session_id()
        self.sequence = 0
        self.plan_id = 0
        self.pose = None
        self.assignment = None
        self.peer_intents = {}
        # cell -> observation validity deadline.  A moved obstacle must not
        # leave the warehouse permanently blocked in this robot's replica.
        self.blockages = {}
        self.width = 90
        self.height = 120
        self.origin_x = -22.5
        self.origin_y = -30.0
        self.static_blocked = set()
        self.lane_waypoints = {}
        self.execution_target = None
        self.execution_task_id = None
        self.execution_waiting = False
        self._reported_assignment_mismatches = set()
        self.docking_target = None
        self._received_local_state = False

        self.corridors = {}
        self.occupied_corridors = {}
        self.peer_tracking = {}

        if map_file:
            self.load_map(map_file)

        self.pub = self.create_publisher(RoutePlan, 'planned_route', FLEET_STATE_QOS)
        self.path_pub = self.create_publisher(Path, 'path', FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(TaskAssignment, 'task_assignment', self.on_assignment, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, 'task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(TrajectoryIntent, '/fleet/trajectory_intent', self.on_intent, PROTOCOL_QOS)
        self.create_subscription(BlockageObservation, '/fleet/blockage_observation', self.on_blockage, PROTOCOL_QOS)
        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_corridor, PROTOCOL_QOS)
        self.create_subscription(Pose2D, 'docking/target', lambda msg: setattr(self, 'docking_target', msg), FLEET_STATE_QOS)
        self.create_timer(1.0, self.plan)

    def load_map(self, filename):
        try:
            data = yaml.safe_load(pathlib.Path(filename).read_text())
            (self.resolution, self.width, self.height,
             self.origin_x, self.origin_y,
             self.static_blocked) = map_geometry_from_data(
                data, default_resolution=self.resolution,
                default_width=self.width, default_height=self.height,
                default_origin=(self.origin_x, self.origin_y))
            self.lane_waypoints = lane_waypoint_overrides(
                narrow_lanes(), (self.origin_x, self.origin_y), self.resolution)
            raw_corridors = data.get('mutex_resources', data.get('corridors', {}))
            for name, resource in raw_corridors.items():
                cells = resource.get('cells', []) if isinstance(resource, dict) else resource
                cell_set = set()
                if len(cells) == 2 and isinstance(cells[0], list) and isinstance(cells[1], list):
                    x1, y1 = cells[0]; x2, y2 = cells[1]
                    for x in range(min(x1, x2), max(x1, x2) + 1):
                        for y in range(min(y1, y2), max(y1, y2) + 1):
                            cell_set.add((x, y))
                else:
                    for c in cells: cell_set.add(tuple(c))
                self.corridors[name] = cell_set
            if data.get('generate_narrow_lane_mutex_resources', False):
                zones = data.get('shelf_layout', {}).get('y_zones', {})
                for zone, rows in zones.items():
                    for side, x_start, x_end in (('WEST', -7.5, -21.5), ('EAST', 7.5, 21.5)):
                        for index, (lower, upper) in enumerate(zip(rows, rows[1:]), start=1):
                            y = (float(lower) + float(upper)) / 2.0
                            cy = round((y - self.origin_y) / self.resolution)
                            left = round((min(x_start, x_end) - self.origin_x) / self.resolution)
                            right = round((max(x_start, x_end) - self.origin_x) / self.resolution)
                            self.corridors[f'NC-{zone.upper()}-{side}-{index:02d}'] = {
                                (x, cy) for x in range(left, right + 1)
                            }
                for name, y_start, y_end in (
                    ('NC-MIDDLE-CENTRE', -7.8, 7.8),
                    ('NC-NORTH-CENTRE', 12.2, 29.0),
                ):
                    cx = round((0.0 - self.origin_x) / self.resolution)
                    lower = round((y_start - self.origin_y) / self.resolution)
                    upper = round((y_end - self.origin_y) / self.resolution)
                    self.corridors[name] = {(cx, y) for y in range(lower, upper + 1)}
            self.get_logger().info(f'Loaded map in WhcaPlannerNode: {self.width}x{self.height}, {len(self.static_blocked)} blocked cells, {len(self.corridors)} corridors')
        except Exception as e:
            self.get_logger().error(f'Failed loading map in WhcaPlannerNode: {e}')

    def on_corridor(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            return
        if msg.event == CorridorProtocol.ENTER:
            self.occupied_corridors[msg.fleet_header.robot_id] = msg.corridor_id
        elif msg.event in (CorridorProtocol.EXIT, CorridorProtocol.RELEASE, CorridorProtocol.CANCEL):
            if self.occupied_corridors.get(msg.fleet_header.robot_id) == msg.corridor_id:
                self.occupied_corridors.pop(msg.fleet_header.robot_id, None)

    def on_state(self, msg):
        rid = msg.fleet_header.robot_id
        if rid == self.robot_id:
            self.pose = msg.pose
            return
        speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        now = now_seconds(self)
        prev = self.peer_tracking.get(rid)
        if prev is None or speed >= 0.08:
            self.peer_tracking[rid] = {
                'pose': msg.pose,
                'speed': speed,
                'stopped_since': None,
                'last_seen': now,
            }
        else:
            stopped_since = prev['stopped_since'] if prev.get('stopped_since') is not None else now
            self.peer_tracking[rid] = {
                'pose': msg.pose,
                'speed': speed,
                'stopped_since': stopped_since,
                'last_seen': now,
            }

    def on_local_state(self, msg):
        if not msg.localization_valid:
            return
        self.pose = msg.pose
        if not self._received_local_state:
            self._received_local_state = True
            self.get_logger().info('WHCA planner received first local RobotState sample')

    def on_assignment(self, msg):
        if msg.owner_robot_id != self.robot_id:
            return
        if not msg.active:
            if self.assignment and self.assignment.task.task_id == msg.task.task_id:
                self.assignment = None
            return
        if (self.execution_task_id is not None and
                msg.task.task_id != self.execution_task_id):
            mismatch = (self.execution_task_id, msg.task.task_id)
            if mismatch not in self._reported_assignment_mismatches:
                self._reported_assignment_mismatches.add(mismatch)
                self.get_logger().warning(
                    f'Ignoring assignment {msg.task.task_id}; executor is committed '
                    f'to {self.execution_task_id}')
            return
        self.assignment = msg

    def on_execution(self, msg):
        if msg.owner_robot_id != self.robot_id:
            return
        if msg.phase == TaskExecutionStatus.COMPLETED:
            if self.assignment and self.assignment.task.task_id == msg.task_id:
                self.assignment = None
            self.execution_task_id = None
            self.execution_target = None
            self.execution_waiting = False
            return
        self.execution_task_id = msg.task_id
        self.execution_waiting = msg.phase in (
            TaskExecutionStatus.PICKUP_WAIT,
            TaskExecutionStatus.DROPOFF_WAIT,
            TaskExecutionStatus.COMPLETED
        )
        if self.execution_waiting:
            self.execution_target = None
        else:
            self.execution_target = msg.target

    def on_blockage(self, msg):
        valid_until = stamp_seconds(msg.fleet_header.valid_until)
        if valid_until >= now_seconds(self):
            for cell in msg.cells:
                key = (cell.x, cell.y)
                self.blockages[key] = max(self.blockages.get(key, 0.0), valid_until)

    def on_intent(self, msg):
        if msg.fleet_header.robot_id != self.robot_id and stamp_seconds(msg.fleet_header.valid_until) >= now_seconds(self):
            self.peer_intents[msg.fleet_header.robot_id] = msg

    def to_cell(self, pose):
        cx = round((pose.x - self.origin_x) / self.resolution)
        cy = round((pose.y - self.origin_y) / self.resolution)
        return (max(0, min(cx, self.width - 1)), max(0, min(cy, self.height - 1)))

    def to_pose(self, cell):
        x, y = self.lane_waypoints.get(cell, (
            self.origin_x + cell[0] * self.resolution,
            self.origin_y + cell[1] * self.resolution,
        ))
        return Pose2D(
            x=x,
            y=y,
            theta=0.0
        )

    def plan(self):
        if self.pose is None or (self.assignment is None and self.docking_target is None):
            return

        if (self.assignment is not None and
                self.docking_target is None and
                (self.execution_task_id is None or self.execution_task_id != self.assignment.task.task_id) and
                stamp_seconds(self.assignment.lease_until) < now_seconds(self)):
            return

        if self.execution_waiting and self.docking_target is None:
            # Publish single stationary waypoint while holding at dwell location
            self.sequence += 1
            self.plan_id += 1
            start = self.to_cell(self.pose)
            msg = RoutePlan()
            msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
            msg.plan_id = self.plan_id
            msg.task_id = self.assignment.task.task_id
            msg.route_feasible = True
            msg.failure_reason = 'dwelling at task station'
            msg.cells = [GridCell(x=start[0], y=start[1], time_slot=t) for t in range(self.horizon)]
            msg.waypoints = [self.to_pose(start)]
            self.pub.publish(msg)
            return

        target = self.docking_target or self.execution_target or self.assignment.task.pickup
        start = self.to_cell(self.pose)
        goal = self.to_cell(target)

        now = now_seconds(self)
        self.blockages = {
            cell: valid_until for cell, valid_until in self.blockages.items()
            if valid_until >= now
        }
        reservations = set()
        for intent in list(self.peer_intents.values()):
            if stamp_seconds(intent.fleet_header.valid_until) >= now:
                reservations.update((cell.x, cell.y, cell.time_slot) for cell in intent.reservations)

        # 4D space-time reservations for stationary peers
        for rid, info in self.peer_tracking.items():
            if now - info['last_seen'] < 3.0 and info['stopped_since'] is not None:
                if now - info['stopped_since'] >= 1.0:
                    peer_cell = self.to_cell(info['pose'])
                    for t in range(min(self.horizon, 12)):
                        reservations.add((peer_cell[0], peer_cell[1], t))

        # Global LiDAR reports include this robot as observed by peers.  Never
        # let that coarse representation block the robot's own current
        # footprint. Also leave the validated task-station envelope to the
        # 40 Hz directional safety supervisor. Task
        # stations are validated free-space endpoints; if a real object is at
        # one, local safety will stop before arrival. This prevents a transient
        # quantized endpoint beside the station from making
        # the goal topologically unreachable while preserving distant dynamic
        # obstacle avoidance.
        occupied_corridor_cells = set()
        for peer_id, corridor_name in self.occupied_corridors.items():
            occupied_corridor_cells.update(self.corridors.get(corridor_name, set()))

        planning_blockages = clear_nearfield_blockages(
            set(self.blockages) | occupied_corridor_cells, start, goal,
            clear_goal=True,
            clearance_cells=1,
        )

        heading = self.pose.theta if self.pose is not None else None
        path = whca_star(
            start, goal, self.static_blocked | planning_blockages, reservations,
            self.width, self.height, self.horizon, self.reservation_buffer_cells,
            heading_rad=heading
        )
        task_id = self.assignment.task.task_id if self.assignment else ('docking' if self.docking_target else 'idle')
        if not path:
            self.get_logger().warning(
                f'[{self.robot_id}:WHCA] ERROR: ROUTE_INFEASIBLE for task={task_id}. '
                f'Actor=WhcaPlanner:{self.robot_id}. Info: start={start} '
                f'(static={start in self.static_blocked}, dynamic={start in self.blockages}), '
                f'goal={goal} (static={goal in self.static_blocked}, dynamic={goal in self.blockages}), '
                f'static_cells={len(self.static_blocked)}, dynamic_cells={len(planning_blockages)} '
                f'(raw={len(self.blockages)}), reservations={len(reservations)}. '
                f'Reason=no conflict-free route in current WHCA* window.',
                throttle_duration_sec=3.0,
            )
        else:
            self.get_logger().info(
                f'[{self.robot_id}:WHCA] Decision: ROUTE_FEASIBLE for task={task_id}. '
                f'Actor=WhcaPlanner:{self.robot_id}. Info: start={start} -> goal={goal}, '
                f'steps={len(path)}, waypoints={len(path)}, reservations={len(reservations)}, '
                f'dynamic_cells={len(planning_blockages)}.',
                throttle_duration_sec=5.0,
            )

        self.sequence += 1
        self.plan_id += 1
        msg = RoutePlan()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        msg.plan_id = self.plan_id
        msg.task_id = self.assignment.task.task_id if self.assignment else 'dock_recovery'
        msg.route_feasible = bool(path)
        msg.failure_reason = '' if path else 'no conflict-free route in current WHCA* window'
        msg.cells = [GridCell(x=x, y=y, time_slot=t) for x, y, t in path]
        msg.waypoints = [self.to_pose((x, y)) for x, y, _ in path]
        if path and (path[-1][0], path[-1][1]) == goal and target is not None:
            msg.waypoints[-1] = Pose2D(
                x=float(target.x),
                y=float(target.y),
                theta=float(getattr(target, 'theta', 0.0))
            )
        self.pub.publish(msg)

        nav_path = Path()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = 'map'
        for waypoint in msg.waypoints:
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)
        self.path_pub.publish(nav_path)


def main(args=None):
    rclpy.init(args=args)
    node = WhcaPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

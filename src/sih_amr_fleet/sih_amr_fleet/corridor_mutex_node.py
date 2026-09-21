import pathlib
import uuid
import math
import yaml
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import CorridorProtocol, FleetHealth, PeerTrack, PeerTrackArray, RobotState, RoutePlan
from std_msgs.msg import Bool, Float32, String

from .algorithms import approach_policy
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class CorridorMutexNode(Node):
    """Ricart-Agrawala corridor mutual exclusion node. Network grant never bypasses physical clearance."""

    def __init__(self):
        super().__init__('corridor_mutex_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        map_file = self.declare_parameter('map_file', '').value

        self.session_id = new_session_id()
        self.sequence = 0
        self.clock = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.request = None
        self.deferred = {}  # (corridor_id, request_id) -> requesting_robot_id
        self.peers = {}     # robot_id -> lease_until_s
        self.peer_corridors = {}  # robot_id -> corridor currently announced ENTER
        self.suspect_corridors = set()
        self.entrance_clear = True
        self.corridors = {}
        self.corridor_meta = {}
        self.approach_cells = {}
        self.armed_corridor = None
        self.in_approach = False
        self.communication_degraded = False
        self.approach_distance_m = self.declare_parameter('approach_distance_m', 2.0).value
        self.approach_speed_mps = self.declare_parameter('approach_speed_mps', 0.4).value
        self.last_exited_corridor = None
        self.last_exited_time = 0.0

        if map_file:
            self.load_map(map_file)

        self.pub = self.create_publisher(CorridorProtocol, '/fleet/corridor_protocol', PROTOCOL_QOS)
        self.allowed_pub = self.create_publisher(Bool, 'corridor_motion_allowed', FLEET_STATE_QOS)
        self.speed_pub = self.create_publisher(Float32, 'corridor_speed_cap', FLEET_STATE_QOS)
        self.protected_pub = self.create_publisher(Bool, 'corridor_protected', FLEET_STATE_QOS)

        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_protocol, PROTOCOL_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(PeerTrackArray, 'peer_tracks', self.on_tracks, FLEET_STATE_QOS)
        self.create_subscription(String, 'request_corridor', self.on_request, FLEET_STATE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, POSE_QOS)
        self.create_subscription(Bool, 'entrance_clear', self.on_entrance_clear, FLEET_STATE_QOS)
        self.create_timer(0.1, self.tick)

    def load_map(self, map_file):
        try:
            data = yaml.safe_load(pathlib.Path(map_file).read_text())
            origin = data.get('origin', [0.0, 0.0])
            self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
            self.resolution = float(data.get('resolution_m', self.resolution))

            # Only constrained narrow lanes and narrow junctions use the
            # Ricart-Agrawala mutex.  Spacious main junctions remain WHCA*
            # reservation resources, not one-robot-at-a-time bottlenecks.
            raw_corridors = data.get('mutex_resources', data.get('corridors', {}))
            for name, resource in raw_corridors.items():
                cells = resource.get('cells', []) if isinstance(resource, dict) else resource
                cell_set = set()
                if len(cells) == 2 and isinstance(cells[0], list) and isinstance(cells[1], list):
                    # Bounding endpoints / line segment
                    x1, y1 = cells[0]
                    x2, y2 = cells[1]
                    for x in range(min(x1, x2), max(x1, x2) + 1):
                        for y in range(min(y1, y2), max(y1, y2) + 1):
                            cell_set.add((x, y))
                else:
                    for c in cells:
                        cell_set.add(tuple(c))
                self.corridors[name] = cell_set
            if data.get('generate_narrow_lane_mutex_resources', False):
                # Generate one distinct resource for every shelf-row aisle.
                # This keeps unrelated parallel aisles independent while
                # making every physically narrow aisle one-robot-at-a-time.
                zones = data.get('shelf_layout', {}).get('y_zones', {})
                for zone, rows in zones.items():
                    for side, x_start, x_end in (
                        ('WEST', -7.5, -21.5), ('EAST', 7.5, 21.5),
                    ):
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
            radius_cells = max(1, round(self.approach_distance_m / self.resolution))
            self.corridor_meta = {}
            for name, cells in self.corridors.items():
                xs = [c[0] for c in cells]
                ys = [c[1] for c in cells]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                span_x = (max_x - min_x) * self.resolution
                span_y = (max_y - min_y) * self.resolution
                axis = 'x' if span_x >= span_y else 'y'
                self.corridor_meta[name] = {
                    'axis': axis,
                    'min_x': min_x, 'max_x': max_x,
                    'min_y': min_y, 'max_y': max_y,
                }
                app_cells = set()
                if axis == 'x':
                    for y in range(min_y, max_y + 1):
                        for dx in range(1, radius_cells + 1):
                            app_cells.add((min_x - dx, y))
                            app_cells.add((max_x + dx, y))
                else:
                    for x in range(min_x, max_x + 1):
                        for dy in range(1, radius_cells + 1):
                            app_cells.add((x, min_y - dy))
                            app_cells.add((x, max_y + dy))
                self.approach_cells[name] = app_cells - cells
            self.get_logger().info(f'Loaded {len(self.corridors)} corridors from {map_file}')
        except Exception as e:
            self.get_logger().error(f'Failed loading corridors from {map_file}: {e}')

    def on_entrance_clear(self, msg):
        self.entrance_clear = msg.data

    def on_health(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            self.peers[msg.fleet_header.robot_id] = stamp_seconds(msg.fleet_header.valid_until)

    def on_tracks(self, msg):
        self.communication_degraded = any(track.freshness != PeerTrack.NORMAL for track in msg.tracks)

    def send(self, event, corridor_id, request_id, target=''):
        self.sequence += 1
        self.clock += 1
        msg = CorridorProtocol()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.0)
        msg.corridor_id = corridor_id
        msg.request_id = request_id
        msg.target_robot_id = target
        msg.lamport_time = self.clock
        msg.occupancy_epoch = self.sequence
        msg.event = event
        self.pub.publish(msg)

    def begin_request(self, corridor_id):
        if self.request is not None or not corridor_id:
            return
        request_id = str(uuid.uuid4())
        self.clock += 1
        self.request = {
            'corridor': corridor_id,
            'id': request_id,
            'ts': self.clock,
            'grants': set(),
            'entered': False,
            'was_inside': False,
            'created_at': now_seconds(self)
        }
        self.send(CorridorProtocol.REQUEST, corridor_id, request_id)
        self.get_logger().info(
            f'[{self.robot_id}:CorridorMutex] Decision: REQUEST_MUTEX for corridor {corridor_id}. '
            f'Actor=CorridorMutex:{self.robot_id}. Info: req_id={request_id[:8]}, clock={self.clock}.'
        )

    def on_request(self, msg):
        self.begin_request(msg.data)

    def release_request(self, event):
        """Release the active resource and flush grants deferred behind it."""
        corridor = self.request['corridor']
        request_id = self.request['id']
        event_names = {
            CorridorProtocol.REQUEST: 'REQUEST',
            CorridorProtocol.GRANT: 'GRANT',
            CorridorProtocol.DEFER: 'DEFER',
            CorridorProtocol.ENTER: 'ENTER',
            CorridorProtocol.EXIT: 'EXIT',
            CorridorProtocol.RELEASE: 'RELEASE',
            CorridorProtocol.CANCEL: 'CANCEL',
        }
        event_name = event_names.get(event, str(event))
        self.send(event, corridor, request_id)
        self.get_logger().info(
            f'[{self.robot_id}:CorridorMutex] Decision: RELEASE_MUTEX for corridor {corridor}. '
            f'Actor=CorridorMutex:{self.robot_id}. Event={event_name}, req_id={request_id[:8]}.'
        )
        for (deferred_corridor, deferred_id), peer in list(self.deferred.items()):
            if deferred_corridor == corridor:
                self.send(CorridorProtocol.GRANT, deferred_corridor, deferred_id, peer)
                del self.deferred[(deferred_corridor, deferred_id)]
        self.request = None
        self.armed_corridor = None

    def route_requires_mutex(self, route, corridor_name):
        corridor_cells = self.corridors.get(corridor_name, set())
        if not corridor_cells or not route.cells:
            return False
        intersecting = [c for c in route.cells if (c.x, c.y) in corridor_cells]
        if not intersecting:
            return False
        meta = getattr(self, 'corridor_meta', {}).get(corridor_name, {})
        axis = meta.get('axis', 'x')
        if axis == 'x':
            long_span = (max(c.x for c in intersecting) - min(c.x for c in intersecting)) * self.resolution
            trans_span = (max(c.y for c in intersecting) - min(c.y for c in intersecting)) * self.resolution
        else:
            long_span = (max(c.y for c in intersecting) - min(c.y for c in intersecting)) * self.resolution
            trans_span = (max(c.x for c in intersecting) - min(c.x for c in intersecting)) * self.resolution
        return (long_span >= 1.5 and long_span >= trans_span)

    def on_route(self, route):
        # A grant permits entry; it does not prove that the robot physically
        # entered. Rolling replans can abandon an armed corridor while the
        # robot is still in its approach band. Cancel that unused claim so it
        # cannot retain the corridor speed cap indefinitely.
        if self.request is not None:
            requested_cells = self.corridors.get(self.request['corridor'], set())
            in_requested_approach = (
                getattr(self, 'cell', None) in self.approach_cells.get(self.request['corridor'], set()) or
                self.in_approach
            )
            route_touches_corridor = any((cell.x, cell.y) in requested_cells for cell in route.cells)
            recent_request = (now_seconds(self) - self.request.get('created_at', 0.0) < 5.0)
            inside_corridor = (getattr(self, 'cell', None) in requested_cells)
            still_planned = route.route_feasible and (
                inside_corridor or in_requested_approach or route_touches_corridor or (recent_request and not self.request.get('entered', False)))
            if self.request['was_inside'] or still_planned:
                return
            self.get_logger().info(
                f"[{self.robot_id}:CorridorMutex] Decision: CANCEL_ABANDONED_MUTEX for corridor "
                f"{self.request['corridor']}. Actor=CorridorMutex:{self.robot_id}.")
            self.release_request(CorridorProtocol.CANCEL)
        if not route.route_feasible:
            self.armed_corridor = None
            return
        if self.request is not None:
            self.armed_corridor = self.request['corridor']
            return
        next_corridor = None
        for cell in route.cells[1:]:
            corridor = next((name for name, cells in self.corridors.items() if (cell.x, cell.y) in cells), '')
            if corridor and self.route_requires_mutex(route, corridor):
                next_corridor = corridor
                break
        if next_corridor is None and len(route.cells) == 1:
            cell = route.cells[0]
            corridor = next((name for name, cells in self.corridors.items() if (cell.x, cell.y) in cells), '')
            if corridor and self.route_requires_mutex(route, corridor):
                next_corridor = corridor
        self.armed_corridor = next_corridor

    def on_state(self, state):
        cell = (
            round((state.pose.x - self.origin_x) / self.resolution),
            round((state.pose.y - self.origin_y) / self.resolution)
        )
        self.cell = cell
        now = now_seconds(self)
        recently_exited = (
            self.last_exited_corridor == self.armed_corridor and
            now - self.last_exited_time < 3.0
        )
        self.in_approach = bool(
            self.armed_corridor and not recently_exited and
            cell in self.approach_cells.get(self.armed_corridor, set())
        )
        inside_armed = bool(
            self.armed_corridor and not recently_exited and
            cell in self.corridors.get(self.armed_corridor, set())
        )
        if (self.in_approach or inside_armed) and self.request is None:
            self.begin_request(self.armed_corridor)
        if not self.request or not self.request['entered']:
            return
        corridor_cells = self.corridors.get(self.request['corridor'], set())
        meta = self.corridor_meta.get(self.request['corridor'], {})
        axis = meta.get('axis', 'x')
        in_throat_sweep = False
        if axis == 'x' and 'min_y' in meta:
            y_center = self.origin_y + meta['min_y'] * self.resolution
            x_min_m = self.origin_x + meta['min_x'] * self.resolution
            x_max_m = self.origin_x + meta['max_x'] * self.resolution
            dist_y = abs(state.pose.y - y_center)
            dist_x_ends = min(abs(state.pose.x - x_min_m), abs(state.pose.x - x_max_m))
            if dist_y < 0.8 and dist_x_ends <= 1.5:
                in_throat_sweep = True
        elif axis == 'y' and 'min_x' in meta:
            x_center = self.origin_x + meta['min_x'] * self.resolution
            y_min_m = self.origin_y + meta['min_y'] * self.resolution
            y_max_m = self.origin_y + meta['max_y'] * self.resolution
            dist_x = abs(state.pose.x - x_center)
            dist_y_ends = min(abs(state.pose.y - y_min_m), abs(state.pose.y - y_max_m))
            if dist_x < 0.8 and dist_y_ends <= 1.5:
                in_throat_sweep = True

        inside_corridor = (
            cell in corridor_cells or
            in_throat_sweep or
            any((cx == cell[0] and abs(cy - cell[1]) <= 1) or
                (cy == cell[1] and abs(cx - cell[0]) <= 1)
                for cx, cy in corridor_cells)
        )
        if inside_corridor:
            self.request['was_inside'] = True
        elif self.request['was_inside']:
            # Exited the corridor
            self.get_logger().info(
                f"[{self.robot_id}:CorridorMutex] Decision: EXIT_CORRIDOR {self.request['corridor']}. "
                f"Actor=CorridorMutex:{self.robot_id}. Info: releasing mutex and unblocking deferred peers."
            )
            exited_corridor = self.request['corridor']
            self.release_request(CorridorProtocol.EXIT)
            self.last_exited_corridor = exited_corridor
            self.last_exited_time = now
        elif not self.request['was_inside']:
            entered_at = self.request.get('entered_at', now)
            if now - entered_at > 10.0:
                self.get_logger().warning(
                    f"[{self.robot_id}:CorridorMutex] Decision: TIMEOUT_UNENTERED_MUTEX for corridor "
                    f"{self.request['corridor']}. Actor=CorridorMutex:{self.robot_id}. "
                    f"Held token for {now - entered_at:.1f}s without entering. Releasing to avoid fleet deadlock."
                )
                self.release_request(CorridorProtocol.CANCEL)
            # The exited cell is normally still inside this corridor's broad
            # approach band. Clearing the old arm prevents the next 20 Hz
            # state sample from immediately requesting the corridor again
            # before the 1 Hz rolling route identifies the next resource.

    def on_protocol(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            return
        self.clock = max(self.clock, msg.lamport_time) + 1

        source = msg.fleet_header.robot_id
        if msg.event == CorridorProtocol.REQUEST:
            # Grant immediately if this robot does not claim the corridor, or
            # if the incoming request has strict Lamport precedence and this
            # robot has not yet entered the physical resource.
            incoming = (msg.lamport_time, source)
            mine = (self.request['ts'], self.robot_id) if self.request and self.request['corridor'] == msg.corridor_id else None
            if mine is None or (not self.request.get('entered', False) and incoming < mine):
                self.send(CorridorProtocol.GRANT, msg.corridor_id, msg.request_id, source)
            else:
                self.deferred[(msg.corridor_id, msg.request_id)] = source

        elif msg.event == CorridorProtocol.GRANT and self.request:
            if msg.target_robot_id == self.robot_id and msg.request_id == self.request['id']:
                self.request['grants'].add(msg.fleet_header.robot_id)

        elif msg.event in (CorridorProtocol.RELEASE, CorridorProtocol.CANCEL, CorridorProtocol.EXIT):
            if self.peer_corridors.get(source) == msg.corridor_id:
                self.peer_corridors.pop(source, None)
            for (corridor, request_id), peer in list(self.deferred.items()):
                if corridor == msg.corridor_id:
                    self.send(CorridorProtocol.GRANT, corridor, request_id, peer)
                    del self.deferred[(corridor, request_id)]

        elif msg.event == CorridorProtocol.ENTER:
            # Network ownership is not physical clearance. Retain this fact
            # if the owner later disappears, until an operator/sensor-backed
            # recovery procedure explicitly clears the corridor.
            self.peer_corridors[msg.fleet_header.robot_id] = msg.corridor_id

    def tick(self):
        if self.request is None:
            cap = approach_policy(self.in_approach, False, self.communication_degraded, self.approach_speed_mps)
            self.allowed_pub.publish(Bool(data=math.isinf(cap)))
            self.speed_pub.publish(Float32(data=float(cap if math.isfinite(cap) else 1e6)))
            self.protected_pub.publish(Bool(data=False))
            return

        now = now_seconds(self)
        self.suspect_corridors = {
            corridor_id for robot_id, corridor_id in self.peer_corridors.items()
            if self.peers.get(robot_id, 0.0) < now
        }
        active_peers = {robot for robot, until in self.peers.items() if until >= now}
        suspected = self.request['corridor'] in self.suspect_corridors

        # Once granted and entered, exclusive corridor ownership is secured.
        # Do not allow communication jitter at the approach boundary to revoke
        # permission for an AMR already operating inside the corridor.
        if self.request.get('entered', False):
            permitted = True
        else:
            permitted = (active_peers.issubset(self.request['grants']) and
                         not suspected and
                         not (self.in_approach and self.communication_degraded))

        if permitted and self.entrance_clear and not self.request['entered']:
            self.request['entered'] = True
            self.request['entered_at'] = now_seconds(self)
            self.send(CorridorProtocol.ENTER, self.request['corridor'], self.request['id'])
            self.get_logger().info(
                f"[{self.robot_id}:CorridorMutex] Decision: ENTER_CORRIDOR {self.request['corridor']}. "
                f"Actor=CorridorMutex:{self.robot_id}. Info: full grants received from peers {list(self.request['grants'])}, "
                f"entrance physical clearance confirmed."
            )

        allowed = permitted if self.request.get('entered', False) else (permitted and self.entrance_clear)
        # The conservative speed cap applies in the approach band while
        # entering or waiting for grants. Once inside with exclusive ownership,
        # the AMR tracks at nominal path velocity.
        constrained = self.in_approach and not self.request.get('entered', False)
        cap = approach_policy(
            constrained, allowed, self.communication_degraded,
            self.approach_speed_mps)
        self.allowed_pub.publish(Bool(data=allowed))
        self.speed_pub.publish(Float32(data=float(cap if math.isfinite(cap) else 1e6)))
        self.protected_pub.publish(Bool(data=self.request['entered']))


def main(args=None):
    rclpy.init(args=args)
    node = CorridorMutexNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

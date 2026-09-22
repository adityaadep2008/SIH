"""Pure, ROS-independent algorithms used by the coordination nodes."""
import heapq
import math
import struct


def float32_wire_value(value):
    """Return the exact value a ROS ``float32`` field carries on the wire."""
    return struct.unpack('!f', struct.pack('!f', float(value)))[0]


def lane_waypoint_overrides(lanes, origin, resolution):
    """Map rasterized lane cells to their exact physical centrelines.

    A 0.5 m planning grid cannot represent the warehouse's 1.1554 m shelf-gap
    centrelines exactly.  The discrete cells remain authoritative for WHCA*
    reservations, while these sub-cell waypoint offsets keep the physical AMR
    equally clear of the shelves on both sides of a narrow lane.
    """
    origin_x, origin_y = origin
    overrides = {}
    for lane in lanes:
        start_x = round((lane.start[0] - origin_x) / resolution)
        start_y = round((lane.start[1] - origin_y) / resolution)
        end_x = round((lane.end[0] - origin_x) / resolution)
        end_y = round((lane.end[1] - origin_y) / resolution)
        if start_y == end_y:
            for cell_x in range(min(start_x, end_x), max(start_x, end_x) + 1):
                overrides[(cell_x, start_y)] = (
                    origin_x + cell_x * resolution, lane.start[1])
        elif start_x == end_x:
            for cell_y in range(min(start_y, end_y), max(start_y, end_y) + 1):
                overrides[(start_x, cell_y)] = (
                    lane.start[0], origin_y + cell_y * resolution)
    return overrides


def braking_safe_speed(clearance_m, deceleration_mps2, margin_m):
    """Maximum speed whose ideal stopping distance fits the clearance."""
    clearance = float(clearance_m)
    if math.isinf(clearance):
        return math.inf
    usable = max(0.0, clearance - float(margin_m))
    return math.sqrt(2.0 * max(float(deceleration_mps2), 1e-6) * usable)


def freeze_auction_value(cache, task_id, proposed_value):
    """Return the first value signed for one task's current auction epoch.

    Consensus packets compare the exact advertised value.  A robot may move
    while an auction is open, but its pose-dependent bid must not move with it
    until a new epoch is explicitly started.
    """
    if task_id not in cache:
        cache[task_id] = proposed_value
    return cache[task_id]


class DockLeaseTable:
    """Deterministic replicated dock claims; callers transport its events by DDS.

    The smallest ``(Lamport time, robot id, request id)`` wins a simultaneous
    request.  Entries disappear only after their advertised lease expires or
    a RELEASE/CANCEL event arrives.
    """
    def __init__(self):
        self._offers = {}  # dock -> {(robot, request): (lamport, lease)}

    def observe(self, dock_id, robot_id, request_id, lamport, lease_until, event, now):
        self.expire(now)
        key = (robot_id, request_id)
        if event in ('RELEASE', 'CANCEL'):
            self._offers.get(dock_id, {}).pop(key, None)
            return
        if lease_until > now:
            self._offers.setdefault(dock_id, {})[key] = (int(lamport), float(lease_until))

    def expire(self, now):
        for dock_id in list(self._offers):
            self._offers[dock_id] = {
                key: value for key, value in self._offers[dock_id].items()
                if value[1] > now
            }
            if not self._offers[dock_id]:
                del self._offers[dock_id]

    def owner(self, dock_id, now):
        self.expire(now)
        offers = self._offers.get(dock_id, {})
        if not offers:
            return None
        return min(offers, key=lambda key: (offers[key][0], key[0], key[1]))

    def choose(self, dock_ids, robot_id, now):
        self.expire(now)
        for dock_id in sorted(dock_ids):
            owner = self.owner(dock_id, now)
            if owner is None or owner[0] == robot_id:
                return dock_id
        return None


def approach_policy(in_approach_zone, has_permit, communication_degraded, approach_speed_mps):
    """Return a conservative speed cap for a constrained-resource mouth."""
    if not in_approach_zone:
        return math.inf
    if communication_degraded or not has_permit:
        return 0.0
    return max(0.0, float(approach_speed_mps))


def reverse_recovery_allowed(nearest_obstacle_m, reverse_distance_m,
                             margin_m, inside_protected_resource, dock_claimed):
    """Return whether a reverse recovery move is safe given verified clearances.

    Allows retreat inside protected corridors if rear clearance is verified,
    enabling AMRs to back out of dead-ends or blocked aisles to release mutexes.
    """
    return (not dock_claimed
            and nearest_obstacle_m > reverse_distance_m + margin_m)


def finite_command(linear_x, angular_z):
    """Return whether the final velocity command components are finite."""
    return math.isfinite(float(linear_x)) and math.isfinite(float(angular_z))


def directional_scan_minimum(ranges, angle_min, angle_increment, direction,
                             half_angle, range_min=0.0):
    """Return the nearest valid range inside a scan sector, or infinity.

    A stopping envelope is meaningful only in the commanded travel direction.
    Side and rear returns remain available to higher-level observability but
    must not stop a forward-moving AMR merely because it is parked beside a
    dock, shelf, or wall.
    """
    nearest = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min:
            continue
        angle = angle_min + index * angle_increment
        difference = math.atan2(math.sin(angle - direction), math.cos(angle - direction))
        if abs(difference) <= half_angle:
            nearest = min(nearest, value)
    return nearest


def map_transform_for_anchor(anchor_pose, raw_odom_pose):
    """Return an odom->map transform that maps this raw sample to an anchor."""
    anchor_x, anchor_y, anchor_yaw = anchor_pose
    local_x, local_y, local_yaw = raw_odom_pose
    origin_yaw = anchor_yaw - local_yaw
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    return (anchor_x - cosine * local_x + sine * local_y,
            anchor_y - sine * local_x - cosine * local_y, origin_yaw)


def body_velocity_to_map(linear_x, linear_y, map_yaw):
    """Rotate a base-frame planar velocity into the fleet map frame."""
    cosine, sine = math.cos(map_yaw), math.sin(map_yaw)
    return (
        cosine * linear_x - sine * linear_y,
        sine * linear_x + cosine * linear_y,
    )


def local_point_to_grid_cell(robot_pose, local_point, map_origin, resolution):
    """Transform a base-frame point into an integer planning-grid cell."""
    pose_x, pose_y, pose_yaw = robot_pose
    local_x, local_y = local_point
    cosine, sine = math.cos(pose_yaw), math.sin(pose_yaw)
    world_x = pose_x + cosine * local_x - sine * local_y
    world_y = pose_y + sine * local_x + cosine * local_y
    return (
        round((world_x - map_origin[0]) / resolution),
        round((world_y - map_origin[1]) / resolution),
    )


def sensor_point_to_base(sensor_point, sensor_pose_in_base):
    """Transform a 2-D point from a fixed sensor frame into ``base_link``.

    Gazebo reports LaserScan angles in the LiDAR link frame.  TurtleBot 4 Lite
    mounts ``rplidar_link`` at +pi/2 yaw relative to ``base_link``; treating
    those samples as base-frame points makes stationary obstacles rotate in
    the map whenever the robot turns.
    """
    sensor_x, sensor_y = sensor_point
    offset_x, offset_y, offset_yaw = sensor_pose_in_base
    cosine, sine = math.cos(offset_yaw), math.sin(offset_yaw)
    return (
        offset_x + cosine * sensor_x - sine * sensor_y,
        offset_y + sine * sensor_x + cosine * sensor_y,
    )


def clear_nearfield_blockages(blocked, start, goal=None, clear_goal=False,
                               clearance_cells=1):
    """Let high-rate local safety own the immediate robot/arrival envelope.

    A fleet blockage grid is deliberately coarse.  A peer LiDAR can therefore
    quantize the planning robot's body into its current cell, while a return
    beside a task station can quantize into the goal cell.  Those cells must
    not strand WHCA*: ORCA and the directional safety supervisor still retain
    final authority over real near-field obstacles.
    """
    centres = [start]
    if clear_goal and goal is not None:
        centres.append(goal)
    return {
        cell for cell in blocked
        if all(max(abs(cell[0] - centre[0]), abs(cell[1] - centre[1])) > clearance_cells
               for centre in centres)
    }


def expand_grid_cells(cells, clearance_cells, width, height):
    """Return an in-bounds Chebyshev expansion of a set of grid cells."""
    clearance = max(0, int(clearance_cells))
    return {
        (x + dx, y + dy)
        for x, y in cells
        for dx in range(-clearance, clearance + 1)
        for dy in range(-clearance, clearance + 1)
        if 0 <= x + dx < width and 0 <= y + dy < height
    }


def filter_unexpected_blockages(cells, expected_cells, robot_cells, width, height,
                                 boundary_clearance_cells=1,
                                 robot_clearance_cells=2):
    """Keep only observations that can represent an unmodelled obstacle.

    The global blockage topic must not duplicate known map geometry or other
    AMRs.  Static geometry already constrains WHCA*, while peer reservations,
    ORCA, and the safety supervisor own robot-to-robot separation.  Treating
    their LiDAR silhouettes as expiring map obstacles creates swept trails
    which can topologically close an otherwise free aisle.

    ``expected_cells`` is pre-expanded by the caller because the static map is
    large and does not change at scan rate.  A small boundary envelope removes
    wall returns which quantize just inside the finite planning grid.
    """
    robot_envelope = expand_grid_cells(
        robot_cells, robot_clearance_cells, width, height)
    boundary = max(0, int(boundary_clearance_cells))
    return {
        (int(x), int(y)) for x, y in cells
        if (boundary < int(x) < width - 1 - boundary and
            boundary < int(y) < height - 1 - boundary and
            (int(x), int(y)) not in expected_cells and
            (int(x), int(y)) not in robot_envelope)
    }


def whca_star(start, goal, blocked, reservations, width, height, horizon,
              reservation_buffer_cells=1, heading_rad=None):
    """Return a 4-connected, time-indexed path or an empty list.

    ``reservations`` contains (x, y, time_slot) held by other robots.  At a
    matching time slot, an AMR also avoids the configured Chebyshev-radius
    buffer around each peer reservation (one cell in every direction by
    default). Edge swaps are rejected to prevent head-on passage through one
    grid edge.
    """
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    if goal in blocked:
        return []
    if start in blocked:
        # A continuous pose immediately outside a shelf can round into that
        # shelf's occupied grid cell.  Refusing to plan from that cell leaves
        # a physically collision-free robot permanently stranded.  Snap only
        # the start (never the goal) to the nearest free cardinal cell.
        frontier = [start]
        visited = {start}
        start = None
        while frontier and start is None:
            next_frontier = []
            for x, y in frontier:
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    candidate = (x + dx, y + dy)
                    if candidate in visited:
                        continue
                    visited.add(candidate)
                    if not (0 <= candidate[0] < width and 0 <= candidate[1] < height):
                        continue
                    if candidate not in blocked:
                        start = candidate
                        break
                    next_frontier.append(candidate)
                if start is not None:
                    break
            frontier = next_frontier
        if start is None:
            return []

    def proximity_penalty(x, y):
        # A point-grid shortest path otherwise hugs the first free cell beside
        # a shelf.  Prefer the open main-corridor interior when it is available;
        # genuine narrow aisles remain usable because every alternative there
        # has the same local penalty and physical waypoints are centre-snapped.
        if any((x + dx, y + dy) in blocked
               for dx in range(-1, 2) for dy in range(-1, 2)
               if dx or dy):
            return 4
        if any((x + dx, y + dy) in blocked
               for dx in range(-2, 3) for dy in range(-2, 3)
               if max(abs(dx), abs(dy)) == 2):
            return 1
        return 0

    # Manhattan distance is not a usable rolling-horizon heuristic in this
    # warehouse.  At the end of a shelf row, reaching the next aisle requires
    # temporarily increasing Manhattan distance.  With WAIT as an available
    # action, the old planner preferred twelve waits, returned that stationary
    # window, and made the same choice forever.  Reverse BFS supplies the true
    # static-grid distance.  This reverse Dijkstra search uses the same
    # clearance cost as the forward search; otherwise a robot just outside an
    # unavoidable narrow aisle can still prefer WAIT over entering the aisle.
    distance_to_goal = {goal: 0}
    distance_queue = [(0, goal[0], goal[1])]
    while distance_queue:
        distance, x, y = heapq.heappop(distance_queue)
        if distance != distance_to_goal.get((x, y)):
            continue
        # In reverse, a predecessor enters the current cell during forward
        # travel, so the current cell's proximity cost belongs on this edge.
        next_distance = distance + 1 + proximity_penalty(x, y)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cell = (x + dx, y + dy)
            if (cell in blocked or
                    not (0 <= cell[0] < width and 0 <= cell[1] < height)):
                continue
            if next_distance < distance_to_goal.get(cell, math.inf):
                distance_to_goal[cell] = next_distance
                heapq.heappush(distance_queue, (next_distance, cell[0], cell[1]))
    if start not in distance_to_goal:
        return []

    queue = [(distance_to_goal[start], 0, start[0], start[1], 0)]
    parent = {}
    cost = {(start[0], start[1], 0): 0}

    start_res_dist = {}
    for rx, ry, rt in reservations:
        if rt == 0:
            start_res_dist[(rx, ry)] = max(abs(start[0] - rx), abs(start[1] - ry))

    while queue:
        _, g, x, y, t = heapq.heappop(queue)
        state = (x, y, t)
        if g != cost.get(state):
            continue
        if (x, y) == goal or t >= horizon:
            path = [state]
            while state in parent:
                state = parent[state]
                path.append(state)
            return list(reversed(path))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)):
            nx, ny, nt = x + dx, y + dy, t + 1
            candidate = (nx, ny, nt)
            if not (0 <= nx < width and 0 <= ny < height) or (nx, ny) in blocked:
                continue
            conflict = False
            for rx, ry, rt in reservations:
                if rt == nt:
                    if (nx, ny) == (rx, ry):
                        conflict = True
                        break
                    d_start = start_res_dist.get((rx, ry), math.inf)
                    eff_buf = min(reservation_buffer_cells, max(0, d_start - 1)) if d_start <= reservation_buffer_cells else reservation_buffer_cells
                    if max(abs(nx - rx), abs(ny - ry)) <= eff_buf:
                        conflict = True
                        break
            if conflict or (nx, ny, t) in reservations and (x, y, nt) in reservations:
                continue
            turn_penalty = 0.0
            if (dx, dy) != (0, 0):
                if t == 0 and heading_rad is not None:
                    hx, hy = math.cos(heading_rad), math.sin(heading_rad)
                    step_len = math.hypot(dx, dy)
                    alignment = (dx * hx + dy * hy) / step_len
                    turn_penalty = 1.5 * max(0.0, 1.0 - alignment)
                elif state in parent:
                    pstate = parent[state]
                    pdx, pdy = x - pstate[0], y - pstate[1]
                    if (pdx, pdy) != (0, 0) and (dx, dy) != (pdx, pdy):
                        turn_penalty = 0.35
            new_g = g + 1 + proximity_penalty(nx, ny) + turn_penalty
            if new_g < cost.get(candidate, math.inf):
                cost[candidate] = new_g
                parent[candidate] = state
                h = distance_to_goal[(nx, ny)]
                heapq.heappush(queue, (new_g + h, new_g, nx, ny, nt))
    return []


def static_grid_path_distance(start, goal, blocked, width, height, resolution=0.5):
    """Compute the shortest 2D collision-free grid distance between start and goal in metres.

    Snaps blocked or boundary endpoints to the nearest free grid cell.
    Uses 2D A* search over the static warehouse layout geometry.
    """
    start_cell = (int(start[0]), int(start[1]))
    goal_cell = (int(goal[0]), int(goal[1]))

    if start_cell == goal_cell:
        return 0.0

    def snap_to_free(cell):
        if 0 <= cell[0] < width and 0 <= cell[1] < height and cell not in blocked:
            return cell
        frontier = [cell]
        visited = {cell}
        while frontier:
            next_frontier = []
            for x, y in frontier:
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    cand = (x + dx, y + dy)
                    if cand in visited:
                        continue
                    visited.add(cand)
                    if 0 <= cand[0] < width and 0 <= cand[1] < height and cand not in blocked:
                        return cand
                    if abs(cand[0] - cell[0]) <= 3 and abs(cand[1] - cell[1]) <= 3:
                        next_frontier.append(cand)
            frontier = next_frontier
        return None

    actual_start = snap_to_free(start_cell)
    actual_goal = snap_to_free(goal_cell)
    if actual_start is None or actual_goal is None:
        return math.hypot(goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1]) * resolution

    h_start = math.hypot(actual_goal[0] - actual_start[0], actual_goal[1] - actual_start[1])
    queue = [(h_start, 0.0, actual_start[0], actual_start[1])]
    cost = {actual_start: 0.0}

    while queue:
        _, g, x, y = heapq.heappop(queue)
        if (x, y) == actual_goal:
            return g * resolution
        if g > cost.get((x, y), math.inf):
            continue

        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height) or (nx, ny) in blocked:
                continue
            new_g = g + 1.0
            if new_g < cost.get((nx, ny), math.inf):
                cost[(nx, ny)] = new_g
                h = math.hypot(actual_goal[0] - nx, actual_goal[1] - ny)
                heapq.heappush(queue, (new_g + h, new_g, nx, ny))

    return math.hypot(goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1]) * resolution * 1.5


class ConstantVelocityTrack:
    """Small diagonal constant-velocity Kalman-style filter for peer tracking."""
    def __init__(self, x, y, vx=0.0, vy=0.0):
        self.x, self.y, self.vx, self.vy = x, y, vx, vy
        self.variance = 0.05

    def predict(self, dt, process_noise=0.3):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.variance += process_noise * max(dt, 0.0)

    def update(self, x, y, vx, vy, measurement_variance=0.05):
        gain = self.variance / (self.variance + measurement_variance)
        self.x += gain * (x - self.x)
        self.y += gain * (y - self.y)
        self.vx += gain * (vx - self.vx)
        self.vy += gain * (vy - self.vy)
        self.variance *= (1.0 - gain)


def avoidance_velocity(preferred, self_xy, peers, radius, horizon, max_speed, self_id=None):
    """Reciprocal velocity-obstacle approximation with uncertainty inflation and 2D CPA."""
    vx, vy = preferred
    for peer in peers:
        dx, dy = peer['x'] - self_xy[0], peer['y'] - self_xy[1]
        separation = math.hypot(dx, dy)
        effective_radius = radius + peer.get('radius_inflation', 0.0)
        if separation < 1e-6:
            vx, vy = 0.0, 0.0
            continue

        peer_vx = peer.get('vx', 0.0)
        peer_vy = peer.get('vy', 0.0)
        v_rel_x = peer_vx - vx
        v_rel_y = peer_vy - vy
        v_rel_sq = v_rel_x * v_rel_x + v_rel_y * v_rel_y

        closing = - (dx * v_rel_x + dy * v_rel_y)
        if closing <= 0.0:
            continue

        if v_rel_sq > 1e-6:
            t_cpa = closing / v_rel_sq
            rx_cpa = dx + t_cpa * v_rel_x
            ry_cpa = dy + t_cpa * v_rel_y
            d_cpa = math.hypot(rx_cpa, ry_cpa)
            t_star = max(0.0, min(t_cpa, horizon))
        else:
            t_cpa = horizon
            d_cpa = separation
            t_star = horizon

        rx = dx + t_star * v_rel_x
        ry = dy + t_star * v_rel_y
        d_min = math.hypot(rx, ry)

        if d_min < 2.0 * effective_radius:
            overlap = 2.0 * effective_radius - d_min
            push = overlap / max(t_star, 0.2)
            peer_speed = math.hypot(peer_vx, peer_vy)
            if peer_speed < 0.05:
                # A stationary peer cannot yield; the moving AMR takes full responsibility
                yield_factor = 1.0
            else:
                yield_factor = 0.5

            if yield_factor > 0.0:
                if d_min > 0.01:
                    vx -= yield_factor * push * (rx / d_min)
                    vy -= yield_factor * push * (ry / d_min)
                else:
                    vx -= yield_factor * push * (dx / separation)
                    vy -= yield_factor * push * (dy / separation)

    speed = math.hypot(vx, vy)
    if speed > max_speed:
        vx, vy = vx * max_speed / speed, vy * max_speed / speed
    return vx, vy

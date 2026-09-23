import math
import pathlib
import re
import pytest
import yaml
from sih_amr_fleet.algorithms import (
    ConstantVelocityTrack, DockLeaseTable, approach_policy, avoidance_velocity,
    body_velocity_to_map, braking_safe_speed,
    clear_nearfield_blockages, directional_scan_minimum, expand_grid_cells,
    filter_unexpected_blockages, finite_command, float32_wire_value,
    freeze_auction_value, lane_waypoint_overrides,
    map_transform_for_anchor,
    local_point_to_grid_cell, recovery_yield_priority, reverse_recovery_allowed,
    sensor_point_to_base, static_grid_path_distance, whca_star,
)
from sih_amr_fleet.map_geometry import map_geometry_from_data
from sih_amr_fleet.warehouse_tasks import Lane, aisle_points, narrow_lanes


def test_whca_basic_path():
    path = whca_star(start=(0, 0), goal=(3, 0), blocked=set(), reservations=set(), width=5, height=5, horizon=8)
    assert len(path) == 4
    assert path[0] == (0, 0, 0)
    assert path[-1] == (3, 0, 3)


def test_turtlebot_lite_lidar_point_is_rotated_into_base_link():
    # A scan return along +x of a LiDAR mounted at +pi/2 lies along base +y.
    x, y = sensor_point_to_base((2.0, 0.0), (0.00393584, 0.0, math.pi / 2.0))
    assert x == pytest.approx(0.00393584)
    assert y == pytest.approx(2.0)


def test_nearfield_blockage_filter_clears_self_and_close_goal_envelopes():
    blocked = {(4, 4), (5, 4), (6, 4), (7, 4), (10, 10)}
    assert clear_nearfield_blockages(
        blocked, start=(4, 4), goal=(7, 4), clear_goal=True,
    ) == {(10, 10)}


def test_nearfield_blockage_filter_keeps_distant_goal_blocked():
    blocked = {(4, 4), (5, 4), (7, 4), (10, 10)}
    assert clear_nearfield_blockages(
        blocked, start=(4, 4), goal=(7, 4), clear_goal=False,
    ) == {(7, 4), (10, 10)}


def test_unexpected_blockage_filter_removes_map_peer_and_boundary_echoes():
    expected = expand_grid_cells({(5, 5)}, 1, width=12, height=12)
    observed = {
        (0, 7), (1, 7),       # finite-map boundary/wall envelope
        (4, 5), (6, 6),       # known static geometry halo
        (8, 8), (9, 8),       # peer footprint and quantization envelope
        (3, 9),                # genuine unmodelled persistent obstacle
    }
    assert filter_unexpected_blockages(
        observed, expected, robot_cells={(8, 8)}, width=12, height=12,
        boundary_clearance_cells=1, robot_clearance_cells=1,
    ) == {(3, 9)}


def test_grid_expansion_is_clipped_to_planning_bounds():
    assert expand_grid_cells({(0, 0)}, 1, width=3, height=3) == {
        (0, 0), (0, 1), (1, 0), (1, 1),
    }


def test_cbba_value_is_immutable_within_an_auction_epoch():
    cache = {}
    assert freeze_auction_value(cache, 'task_1', (6.882328, 1)) == (6.882328, 1)
    # Pose motion and asynchronously learned busy state may change a proposed
    # bid, but neither may alter a value already signed in this epoch.
    assert freeze_auction_value(cache, 'task_1', (6.884371, 1)) == (6.882328, 1)
    assert freeze_auction_value(cache, 'task_1', (1.0e9, 1)) == (6.882328, 1)
    assert freeze_auction_value(cache, 'task_2', (1.0e9, 1)) == (1.0e9, 1)


def test_cbba_bid_is_canonicalized_to_its_float32_wire_value():
    assert float32_wire_value(22.016171488229702) == 22.016172409057617


def test_narrow_lane_waypoints_use_exact_subcell_centreline():
    lane = Lane('test', 'narrow', (7.5, -27.7725), (21.5, -27.7725))
    overrides = lane_waypoint_overrides([lane], (-22.5, -30.0), 0.5)
    assert overrides[(73, 4)] == (14.0, -27.7725)
    assert overrides[(85, 4)] == (20.0, -27.7725)


def test_braking_speed_cap_fits_available_clearance():
    assert braking_safe_speed(1.25, 0.8, 0.25) == pytest.approx(
        math.sqrt(1.6))
    assert braking_safe_speed(0.20, 0.8, 0.25) == 0.0
    assert math.isinf(braking_safe_speed(math.inf, 0.8, 0.25))


def test_whca_avoids_reserved_cell():
    # Cell (3, 2) is reserved at time slot 1.
    path = whca_star(start=(2, 2), goal=(5, 2), blocked=set(), reservations={(3, 2, 1)}, width=7, height=5, horizon=10)
    assert path
    assert (3, 2, 1) not in path
    # Robot will wait or route around
    assert path[-1][0] == 5 and path[-1][1] == 2


def test_whca_avoids_one_cell_reservation_buffer():
    # The peer owns (1, 1) at t=1.  The direct first step (1, 2) is adjacent,
    # so the default one-cell safety buffer must rule it out too.
    path = whca_star(
        start=(0, 2), goal=(3, 2), blocked=set(), reservations={(1, 1, 1)},
        width=5, height=5, horizon=10,
    )
    assert path
    assert (1, 2, 1) not in path


def test_whca_prevents_edge_swap():
    # Peer moving from (1, 0) at t=0 to (0, 0) at t=1.
    # Self moving from (0, 0) at t=0 to (1, 0) at t=1 would be an edge swap.
    peer_reservations = {(3, 2, 0), (2, 2, 1)}
    path = whca_star(start=(2, 2), goal=(4, 2), blocked=set(), reservations=peer_reservations, width=6, height=6, horizon=8, reservation_buffer_cells=0)
    assert path
    # First step should NOT be (1, 0, 1) because that would head-on swap with peer
    assert path[1] != (3, 2, 1)


def test_whca_respects_static_obstacles():
    blocked = {(1, 0), (1, 1), (1, 2)}
    path = whca_star(start=(0, 0), goal=(2, 0), blocked=blocked, reservations=set(), width=5, height=5, horizon=10)
    assert path
    for x, y, t in path:
        assert (x, y) not in blocked


def test_whca_snaps_a_rounded_blocked_start_to_free_space():
    # A continuous robot pose can still be physically outside a shelf while
    # rounding to the shelf's occupied 0.5 m cell.  Planning must recover from
    # that representation boundary instead of remaining infeasible forever.
    path = whca_star(
        start=(1, 1), goal=(4, 1), blocked={(1, 1)}, reservations=set(),
        width=6, height=4, horizon=8,
    )
    assert path
    assert path[0][:2] == (2, 1)
    assert path[-1][:2] == (4, 1)
    assert all((x, y) != (1, 1) for x, y, _ in path)


def test_whca_does_not_snap_an_invalid_blocked_goal():
    assert whca_star(
        start=(0, 0), goal=(2, 0), blocked={(2, 0)}, reservations=set(),
        width=4, height=4, horizon=6,
    ) == []


def test_whca_horizon_limit():
    # Goal is far away, horizon is only 3 steps
    path = whca_star(start=(0, 0), goal=(10, 10), blocked=set(), reservations=set(), width=20, height=20, horizon=3)
    assert len(path) == 4  # t=0, t=1, t=2, t=3
    assert path[-1][2] == 3


def test_whca_rolling_window_detours_around_shelf_instead_of_waiting_forever():
    # The goal is directly north, but a wide shelf forces an initially
    # Manhattan-worsening west/east detour longer than the rolling horizon.
    # A Manhattan heuristic plus WAIT returned a stationary window forever.
    shelf = {(x, y) for x in range(10, 19) for y in range(5, 8)}
    path = whca_star(
        start=(14, 4), goal=(14, 9), blocked=shelf, reservations=set(),
        width=30, height=20, horizon=12,
    )
    assert path
    assert path[-1][:2] != (14, 4)
    assert all((x, y) not in shelf for x, y, _ in path)


def test_whca_prefers_clearance_over_hugging_a_long_obstacle_edge():
    wall = {(4, y) for y in range(1, 12)}
    path = whca_star(
        start=(5, 0), goal=(5, 13), blocked=wall, reservations=set(),
        width=10, height=15, horizon=13,
    )
    assert path
    assert any(x >= 7 for x, _, _ in path)


def test_locked_task_points_are_free_in_the_shared_static_geometry():
    package_root = pathlib.Path(__file__).parents[1]
    data = yaml.safe_load(package_root.joinpath('maps/demo_warehouse.yaml').read_text())
    resolution, _, _, origin_x, origin_y, blocked = map_geometry_from_data(data)
    for point in aisle_points():
        cell = (
            round((point.x - origin_x) / resolution),
            round((point.y - origin_y) / resolution),
        )
        assert cell not in blocked


def test_track_prediction_increases_uncertainty():
    track = ConstantVelocityTrack(0.0, 0.0, 1.0, 0.0)
    before = track.variance
    track.predict(1.0)
    assert track.x == 1.0
    assert track.variance > before


def test_track_update_reduces_uncertainty():
    track = ConstantVelocityTrack(0.0, 0.0, 1.0, 0.0)
    track.predict(2.0)
    cov_after_predict = track.variance
    track.update(2.1, 0.05, 1.0, 0.0)
    assert track.variance < cov_after_predict


def test_avoidance_reduces_head_on_speed():
    safe = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.4, 'y': 0.0, 'vx': -0.2, 'vy': 0.0, 'radius_inflation': 0.0}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    assert safe[0] < 0.4


def test_avoidance_uncertainty_inflation():
    # With larger uncertainty inflation, repulsive force is stronger (lower forward velocity and stronger push)
    safe_small_cov = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.8, 'y': 0.1, 'vx': -0.1, 'vy': 0.0, 'radius_inflation': 0.0}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    safe_large_cov = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.8, 'y': 0.1, 'vx': -0.1, 'vy': 0.0, 'radius_inflation': 0.3}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    # Larger inflation produces a stronger repulsive push (forward velocity is reduced or reversed)
    assert safe_large_cov[0] < safe_small_cov[0]
    # Lateral push is also stronger
    assert abs(safe_large_cov[1]) > abs(safe_small_cov[1])


def test_avoidance_clips_max_speed():
    safe = avoidance_velocity(
        preferred=(0.8, 0.8),
        self_xy=(0.0, 0.0),
        peers=[],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    speed = math.hypot(safe[0], safe[1])
    assert speed <= 0.45 + 1e-6


def test_avoidance_2d_cpa_parallel_clearance():
    safe = avoidance_velocity(
        preferred=(0.0, 0.46),
        self_xy=(-10.20, -8.0),
        peers=[{
            'x': -9.40, 'y': -8.0,
            'vx': 0.0, 'vy': -0.46,
            'radius_inflation': 0.04,
            'id': 'robot_2'
        }],
        radius=0.28,
        horizon=1.5,
        max_speed=0.46,
        self_id='robot_1'
    )
    assert abs(safe[1] - 0.46) < 0.01
    assert abs(safe[0]) < 0.01


def test_avoidance_deterministic_priority_head_on():
    # In balanced reciprocal ORCA (per Rule 4 of AGENTS.md), opposing moving peers
    # both reciprocally decelerate to avoid collision without asymmetric ID overrides.
    robot1_safe = avoidance_velocity(
        preferred=(0.46, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{
            'x': 1.5, 'y': 0.0,
            'vx': -0.46, 'vy': 0.0,
            'radius_inflation': 0.01,
            'id': 'robot_3'
        }],
        radius=0.35,
        horizon=1.5,
        max_speed=0.46,
        self_id='robot_1'
    )
    # Reciprocally decelerated below preferred speed
    assert robot1_safe[0] < 0.46

    robot3_safe = avoidance_velocity(
        preferred=(-0.46, 0.0),
        self_xy=(1.5, 0.0),
        peers=[{
            'x': 0.0, 'y': 0.0,
            'vx': 0.46, 'vy': 0.0,
            'radius_inflation': 0.01,
            'id': 'robot_1'
        }],
        radius=0.35,
        horizon=1.5,
        max_speed=0.46,
        self_id='robot_3'
    )
    # Reciprocally decelerated symmetrically
    assert robot3_safe[0] > -0.46
    assert abs(abs(robot1_safe[0]) - abs(robot3_safe[0])) < 1e-4


def test_orca_node_clamping_and_priority_halt():
    from sih_amr_fleet.orca_node import OrcaNode
    from geometry_msgs.msg import Pose2D, Twist
    from sih_amr_interfaces.msg import PeerTrack, PeerTrackArray, RobotState
    import rclpy

    shutdown_after = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_after = True

    try:
        node = OrcaNode()
        node.robot_id = 'robot_2'
        state = RobotState()
        state.localization_valid = True
        state.pose = Pose2D(x=1.0, y=0.0, theta=0.0)
        node.on_local_state(state)

        peer = PeerTrack()
        peer.robot_id = 'robot_1'
        peer.pose.x = 2.0
        peer.pose.y = 0.0
        peer.twist.linear.x = -0.46
        peer.covariance_trace = 0.01

        peers_msg = PeerTrackArray()
        peers_msg.tracks = [peer]
        node.tracks = peers_msg.tracks

        node.desired.linear.x = 0.46
        node.desired.angular.z = 0.0

        published = []
        node.pub = type('MockPub', (), {'publish': lambda self, msg: published.append(msg)})()

        node.control()
        assert published[-1].linear.x >= 0.0, 'Speed cannot be negative!'
        assert published[-1].linear.x <= 0.46, 'Speed cannot exceed desired!'

        # Zero desired speed produces zero output
        node.desired.linear.x = 0.0
        node.control()
        assert published[-1].linear.x == 0.0, 'Zero desired must produce zero output!'

        # Reverse retreat velocity passes through safely
        node.desired.linear.x = -0.20
        node.control()
        assert published[-1].linear.x == -0.20, 'Negative retreat velocity must pass through!'

        node.destroy_node()
    finally:
        if shutdown_after and rclpy.ok():
            rclpy.shutdown()


def test_predefined_narrow_lanes_cover_every_storage_aisle():
    lanes = narrow_lanes()
    assert len(lanes) == 48
    assert all(lane.kind == 'narrow' for lane in lanes)
    assert any(lane.lane_id == 'NC-MIDDLE-CENTRE' for lane in lanes)
    assert any(point.x == 0.0 for point in aisle_points())


def test_dock_simultaneous_requests_have_one_lamport_owner_and_alternate():
    table = DockLeaseTable()
    table.observe('charging_pad_1', 'robot_2', 'r2', 10, 5.0, 'REQUEST', 0.0)
    table.observe('charging_pad_1', 'robot_1', 'r1', 10, 5.0, 'REQUEST', 0.0)
    assert table.owner('charging_pad_1', 0.1) == ('robot_1', 'r1')
    assert table.choose(['charging_pad_1', 'charging_pad_2'], 'robot_2', 0.1) == 'charging_pad_2'


def test_dock_lease_expiry_and_release_make_pad_available():
    table = DockLeaseTable()
    table.observe('charging_pad_1', 'robot_1', 'r1', 1, 2.0, 'CLAIM', 0.0)
    assert table.owner('charging_pad_1', 1.0) == ('robot_1', 'r1')
    table.observe('charging_pad_1', 'robot_1', 'r1', 2, 0.0, 'RELEASE', 1.1)
    assert table.owner('charging_pad_1', 1.2) is None
    table.observe('charging_pad_1', 'robot_1', 'r1', 3, 2.0, 'CLAIM', 0.0)
    assert table.owner('charging_pad_1', 2.1) is None


def test_narrow_approach_stops_without_permit_or_when_network_degraded():
    assert approach_policy(True, False, False, 0.4) == 0.0
    assert approach_policy(True, True, True, 0.4) == 0.0
    assert approach_policy(True, True, False, 0.4) == 0.4
    assert math.isinf(approach_policy(False, False, True, 0.4))


def test_recovery_reverse_rejects_obstructed_retreat_allows_protected_when_clear():
    # When rear clearance is verified (3.0m > 2.0m + 0.5m), retreat is allowed
    assert reverse_recovery_allowed(3.0, 2.0, 0.5, False, False)
    # When rear clearance is insufficient (2.5m <= 2.0m + 0.5m), retreat is rejected
    assert not reverse_recovery_allowed(2.5, 2.0, 0.5, False, False)
    # When inside protected corridor, retreat IS allowed if rear clearance is verified (to back out of dead-ends)
    assert reverse_recovery_allowed(5.0, 2.0, 0.5, True, False)
    # When in final docking (dock_claimed=True), retreat is rejected to preserve docking alignment
    assert not reverse_recovery_allowed(5.0, 2.0, 0.5, False, True)


def test_recovery_yield_priority_protects_corridor_owner_and_breaks_open_space_symmetry():
    # Protected corridor owner NEVER yields to outside peers (even if peer ID is smaller, e.g. robot_3 vs robot_4)
    assert not recovery_yield_priority(inside_protected_resource=True, conflicting_peer='robot_3', robot_id='robot_4')
    assert not recovery_yield_priority(inside_protected_resource=True, conflicting_peer='robot_1', robot_id='robot_4')
    assert not recovery_yield_priority(inside_protected_resource=True, conflicting_peer=None, robot_id='robot_4')

    # Unprotected / open-space robots break symmetry deterministically:
    # Lower-priority robot (robot_4 > robot_3) yields to robot_3
    assert recovery_yield_priority(inside_protected_resource=False, conflicting_peer='robot_3', robot_id='robot_4')
    # Higher-priority robot (robot_1 < robot_2) does NOT yield to robot_2
    assert not recovery_yield_priority(inside_protected_resource=False, conflicting_peer='robot_2', robot_id='robot_1')
    # If no conflicting peer specified in open space, defaults to yielding to clear obstacle
    assert recovery_yield_priority(inside_protected_resource=False, conflicting_peer=None, robot_id='robot_1')


def test_map_declares_dock_resources_and_strip_references():
    data = yaml.safe_load(pathlib.Path(__file__).parents[1].joinpath('maps/demo_warehouse.yaml').read_text())
    assert set(data['dock_resources']) == set(data['anchors'])
    assert all('centre_strip' in resource for resource in data['dock_resources'].values())


def test_confirmed_dock_anchor_transform_corrects_current_raw_odom_sample():
    origin_x, origin_y, origin_yaw = map_transform_for_anchor((3.6, -29.55, -1.57), (1.0, 0.5, 0.2))
    actual_x = origin_x + math.cos(origin_yaw) * 1.0 - math.sin(origin_yaw) * 0.5
    actual_y = origin_y + math.sin(origin_yaw) * 1.0 + math.cos(origin_yaw) * 0.5
    assert actual_x == pytest.approx(3.6)
    assert actual_y == pytest.approx(-29.55)
    assert origin_yaw + 0.2 == pytest.approx(-1.57)


def test_body_forward_velocity_is_rotated_into_map_frame():
    map_vx, map_vy = body_velocity_to_map(1.0, 0.0, math.pi / 2.0)
    assert map_vx == pytest.approx(0.0, abs=1e-9)
    assert map_vy == pytest.approx(1.0)


def test_local_obstacle_point_uses_robot_yaw_and_map_grid_coordinates():
    # A point 1 m forward from a north-facing robot at map (3, -8) is world
    # (3, -7), which is planning cell (51, 46) in the warehouse grid.
    assert local_point_to_grid_cell(
        (3.0, -8.0, math.pi / 2.0), (1.0, 0.0), (-22.5, -30.0), 0.5
    ) == (51, 46)


def test_telemetry_schema_explicitly_keeps_map_and_raw_simulator_odom_distinct():
    source = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet/data_collection_node.py').read_text()
    assert "'map_pose'" in source
    assert "'gazebo_odom'" in source
    assert "'schema_version': '0.6.0'" in source
    assert "record['wall_logged_at'] = time.time()" in source
    assert 'self.last_assignments = {}' in source
    assert 'self.last_execution_sequences = {}' in source


def test_live_qos_matches_dock_protocol_and_nearest_obstacle_publishers():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    localization = package_root.joinpath('localization_node.py').read_text()
    follower = package_root.joinpath('path_follower_node.py').read_text()
    assert "DockProtocol, '/fleet/dock_protocol', self.on_dock_protocol, PROTOCOL_QOS" in localization
    assert "Float32, 'nearest_obstacle_m', lambda msg: setattr(self, 'nearest', msg.data), POSE_QOS" in follower


def test_safety_rejects_nonfinite_velocity_components():
    assert finite_command(0.0, 0.0)
    assert not finite_command(float('nan'), 0.0)
    assert not finite_command(0.0, float('inf'))


def test_directional_scan_braking_ignores_side_and_rear_returns():
    # Four beams: front, left, rear, right.  A parked AMR may be close to a
    # dock or wall at its side/rear, but only the travel sector can veto a
    # forward command.
    ranges = [2.0, 0.34, 0.20, 0.34]
    assert directional_scan_minimum(ranges, 0.0, math.pi / 2.0, 0.0, math.pi / 4.0) == pytest.approx(2.0)
    assert directional_scan_minimum(ranges, 0.0, math.pi / 2.0, math.pi, math.pi / 4.0) == pytest.approx(0.20)
    assert math.isinf(directional_scan_minimum([float('nan')], 0.0, 1.0, 0.0, 1.0))


def test_cbba_has_a_verified_fleet_state_fallback_and_exact_local_state_qos():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    cbba = package_root.joinpath('cbba_node.py').read_text()
    localization = package_root.joinpath('localization_node.py').read_text()
    assert "RobotState, '/fleet/robot_state', self.on_fleet_state, FLEET_STATE_QOS" in cbba
    assert "create_publisher(RobotState, 'state', POSE_QOS)" in localization


def test_task_sources_replay_pending_work_and_generator_waits_for_fleet_readiness():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    scenario = package_root.joinpath('task_scenario_node.py').read_text()
    assert 'def fleet_ready(self):' in generator
    assert 'return self.expected_robot_ids.issubset(self.ready_robots)' in generator
    assert "TASK_SOURCE_QOS" in generator
    assert 'def _publish_pending(self, now):' in generator
    assert "self.active_tasks[task_id] = (announcement, now)" in generator
    assert 'self.pending[spec[\'id\']] = (msg, now)' in scenario
    assert 'TASK_SOURCE_QOS' in scenario


def test_task_source_fanout_and_local_execution_paths_are_resilient_to_shared_dds_loss():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    cbba = package_root.joinpath('cbba_node.py').read_text()
    executor = package_root.joinpath('task_execution_node.py').read_text()
    planner = package_root.joinpath('whca_planner_node.py').read_text()
    collector = package_root.joinpath('data_collection_node.py').read_text()
    assert "String, f'/{robot_id}/task_inbox', FLEET_STATE_QOS" in generator
    assert "String, f'/{self.robot_id}/task_inbox', self.on_local_task_wire, FLEET_STATE_QOS" in cbba
    assert "TaskExecutionStatus, 'task_execution_status', FLEET_STATE_QOS" in executor
    assert "TaskExecutionStatus, 'task_execution_status', self.on_execution, FLEET_STATE_QOS" in planner
    assert 'seen_task_announcements' in collector
    assert "RoutePlan, f'/{robot_id}/planned_route', self.on_route" in collector
    assert "TaskAssignment, f'/{robot_id}/task_assignment', self.on_assignment" in collector


def test_random_baseline_uses_one_authoritative_acknowledged_fanout_source():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    cbba = package_root.joinpath('cbba_node.py').read_text()
    launch = pathlib.Path(__file__).parents[1].joinpath('launch/fleet.launch.py').read_text()
    assert 'json.dumps' in generator
    assert "'/fleet/task_wire'" in generator
    assert "name='random_task_generator_node'" in launch
    assert 'task_receipt_pub' in cbba
    assert "'/fleet/task_receipt', self.on_task_receipt, FLEET_STATE_QOS" in generator
    assert 'Task delivery confirmed for' in generator
    assert 'def task_transport_ready(self):' in generator
    assert 'Waiting for task inbox readers' in generator
    assert 'created_at_ns' in generator and 'expires_at_ns' in generator
    assert 'created_at_ns' in cbba and 'expires_at_ns' in cbba


def test_cbba_requires_matching_claims_from_every_expected_participant():
    cbba = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet/cbba_node.py').read_text()
    assert "'expected_robot_ids', ['robot_1', 'robot_2', 'robot_3', 'robot_4']" in cbba
    assert 'self.consensus_participants.setdefault(msg.task_id, set()).add' in cbba
    assert 'self.bid_views = {}' in cbba
    assert 'self.claim_views = {}' in cbba
    assert 'self.own_bids = {}' in cbba
    assert 'self.own_claims = {}' in cbba
    assert 'freeze_auction_value(\n                self.own_bids' in cbba
    assert 'freeze_auction_value(self.own_claims' in cbba
    assert 'missing = self.expected_robot_ids - set(views)' in cbba
    assert 'if missing or not settled:' in cbba
    assert 'missing_claims = self.expected_robot_ids - set(claims)' in cbba
    assert 'not missing_claims and not mismatched_claims' in cbba
    assert 'self.claim_key(claims[source]) != claim_key' in cbba
    assert 'views[source].fleet_header.session_id' in cbba
    assert 'claims[source].fleet_header.sequence_no >' not in cbba
    assert 'own_bid_value, own_bid_epoch = freeze_auction_value' in cbba
    assert 'msg.winning_bid = float32_wire_value(winning_bid)' in cbba
    assert 'task.task_id not in self.execution_confirmed_tasks' in cbba
    assert 'self.publish_consensus(bid_refresh)' in cbba
    assert 'claim_values={claim_values}' in cbba
    assert 'if winner == self.robot_id and winning_bid < UNAVAILABLE_BID' in cbba


def test_cbba_consensus_has_independently_matched_targeted_transport():
    cbba = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/cbba_node.py').read_text()
    assert "String, f'/{robot_id}/consensus_inbox', PROTOCOL_QOS" in cbba
    assert "String, f'/{self.robot_id}/consensus_inbox'" in cbba
    assert 'def consensus_wire_payload(self, consensus):' in cbba
    assert 'def on_local_consensus_wire(self, msg):' in cbba
    assert 'self.publish_consensus(bid_msg)' in cbba
    assert 'self.publish_consensus(claim)' in cbba
    assert 'old.fleet_header.sequence_no >= msg.fleet_header.sequence_no' in cbba


def test_execution_commits_cbba_owner_and_planner_task_identity():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    cbba = package_root.joinpath('cbba_node.py').read_text()
    planner = package_root.joinpath('whca_planner_node.py').read_text()
    assert 'self.executing_tasks = {}' in cbba
    assert 'self.busy_robots = {}' in cbba
    assert "Ignoring conflicting executor owner" in cbba
    assert 'msg.winner_robot_id != committed[0]' in cbba
    assert 'self.committed_claims[task.task_id] = claim_key' in cbba
    assert 'self.execution_task_id = None' in planner
    assert 'msg.task.task_id != self.execution_task_id' in planner
    assert 'if msg.phase == TaskExecutionStatus.COMPLETED:' in planner
    assert 'self.assignment = None' in planner


def test_committed_work_survives_announcement_ttl_until_completion():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    cbba = package_root.joinpath('cbba_node.py').read_text()
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    assert 'task.task_id not in self.executing_tasks' in cbba
    assert 'self.executing_tasks = set()' in generator
    assert 'if task_id in self.executing_tasks:' in generator


def test_blockage_detector_excludes_shared_static_occupancy():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    detector = package_root.joinpath('blockage_detector_node.py').read_text()
    assert 'map_geometry_from_data' in detector
    assert 'self.expected_cells = expand_grid_cells' in detector
    assert 'filter_unexpected_blockages' in detector
    assert "RobotState, '/fleet/robot_state', self.on_fleet_state" in detector
    assert "'blockage_observation_ttl_s', 0.75" in detector


def test_corridor_exit_clears_stale_route_arm():
    source = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/corridor_mutex_node.py').read_text()
    release_path = source[source.index('def release_request'):source.index('def on_route')]
    assert release_path.index('self.request = None') < release_path.index('self.armed_corridor = None')
    assert 'self.release_request(CorridorProtocol.EXIT)' in source


def test_corridor_speed_cap_applies_in_approach_band_before_entry():
    source = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/corridor_mutex_node.py').read_text()
    assert "constrained = self.in_approach and not self.request.get('entered', False)" in source


def test_corridor_claim_is_cancelled_when_rolling_route_abandons_it():
    source = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/corridor_mutex_node.py').read_text()
    route_path = source[source.index('def on_route'):source.index('def on_state')]
    assert "if self.request['was_inside'] or still_planned" in route_path
    assert 'self.release_request(CorridorProtocol.CANCEL)' in route_path
    assert 'for cell in route.cells[1:]' in route_path


def test_stale_routes_cannot_be_refreshed_as_ghost_reservations():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    reservations = package_root.joinpath('reservation_manager_node.py').read_text()
    follower = package_root.joinpath('path_follower_node.py').read_text()
    assert 'self.current = route if route.route_feasible else None' in reservations
    assert 'stamp_seconds(self.current.fleet_header.valid_until) < now_seconds(self)' in reservations
    assert 'self.route.task_id != msg.task_id' in follower


def test_data_collector_correlates_every_motion_pipeline_gate():
    collector = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/data_collection_node.py').read_text()
    assert "'event_type': 'pipeline_diagnostic'" in collector
    assert "('cmd_vel_desired', 'desired_command')" in collector
    assert "('cmd_vel_candidate', 'orca_command')" in collector
    assert "('cmd_vel', 'final_command')" in collector
    assert "'CORRIDOR_MOTION_DENIED'" in collector
    assert "'ORCA_LINEAR_VETO'" in collector
    assert "'SAFETY_LINEAR_VETO'" in collector
    assert "'ACTUATION_NOT_FOLLOWING_COMMAND'" in collector
    assert "'cells': [[int(cell.x), int(cell.y)] for cell in msg.cells]" in collector
    assert "self.create_subscription(Log, '/rosout', self.on_ros_log, ROSOUT_QOS)" in collector
    assert "'event_type': 'ros_log'" in collector
    assert "if int(msg.level) < int(Log.WARN):" in collector


def test_fleet_sensor_profile_removes_unused_gpu_sensors():
    source = pathlib.Path(__file__).parents[1].joinpath(
        'sih_amr_fleet/fleet_robot_description.py').read_text()
    assert "FLEET_SENSOR_NAMES = {'rplidar', 'bumper_contact_sensor'}" in source
    assert "sensor_profile == 'fleet'" in source
    assert 'parent.remove(sensor)' in source
    assert "visualize.text = 'false'" in source


def test_simulation_time_and_tracking_speed_are_launch_configurable():
    package_root = pathlib.Path(__file__).parents[1]
    launch = package_root.joinpath('launch/fleet.launch.py').read_text()
    reservations = package_root.joinpath('sih_amr_fleet/reservation_manager_node.py').read_text()
    launcher = pathlib.Path(__file__).parents[3].joinpath('scripts/launch_four_amrs.sh').read_text()
    assert "'use_sim_time': True" in launch
    assert "DeclareLaunchArgument('path_tracking_speed_mps'" in launch
    assert 'derived_dt = grid_resolution / max(tracking_speed, 0.01)' in reservations
    assert 'FLEET_TRACKING_SPEED_MPS="${FLEET_TRACKING_SPEED_MPS:-0.46}"' in launcher
    assert 'sensor_profile:="$SENSOR_PROFILE"' in launcher


def test_four_amr_launcher_defaults_to_cyclone_and_keeps_fastdds_override_safe():
    launcher = pathlib.Path(__file__).parents[3].joinpath('scripts/launch_four_amrs.sh').read_text()
    assert 'RMW_IMPLEMENTATION="${SIH_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"' in launcher
    assert 'CYCLONEDDS_URI="${SIH_CYCLONEDDS_URI:-file://$SIH_ROOT/src/sih_amr_fleet/config/cyclonedds.xml}"' in launcher
    assert 'if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps* ]]' in launcher
    assert 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4' in launcher
    assert 'kill -TERM "$pid"' in launcher


def test_cyclone_config_allows_the_full_fleet_participant_graph():
    config = pathlib.Path(__file__).parents[1].joinpath('config/cyclonedds.xml').read_text()
    match = re.search(r'<MaxAutoParticipantIndex>(\d+)</MaxAutoParticipantIndex>', config)
    assert match is not None and int(match.group(1)) >= 119


def test_controller_contract_is_stamped_and_bridge_rejects_nonfinite_commands():
    package_root = pathlib.Path(__file__).parents[1]
    bridge = package_root.joinpath('sih_amr_fleet/twist_stamper_node.py').read_text()
    control = package_root.joinpath('config/fleet_fast_control.yaml').read_text()
    assert 'use_stamped_vel: true' in control
    assert "TwistStamped, 'diffdrive_controller/cmd_vel'" in bridge
    assert 'Rejected nonfinite cmd_vel before controller bridge' in bridge
    assert 'def publish_idle_stop(self):' in bridge
    assert 'durability=DurabilityPolicy.TRANSIENT_LOCAL' in bridge
    assert 'self.create_timer(0.05, self.publish_idle_stop)' in bridge


def test_local_state_fallback_covers_every_motion_critical_node():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    for name in ('task_execution_node.py', 'whca_planner_node.py', 'path_follower_node.py',
                 'orca_node.py', 'safety_supervisor_node.py'):
        source = package_root.joinpath(name).read_text()
        assert "RobotState, 'state', self.on_local_state, POSE_QOS" in source
    safety = package_root.joinpath('safety_supervisor_node.py').read_text()
    assert 'directional_scan_minimum' in safety
    assert "'scan stale'" in safety
    assert "self.margin = self.declare_parameter('braking_margin_m'" in safety
    assert "self.forward_half_angle = self.declare_parameter('braking_sector_half_angle_rad'" in safety


def test_static_grid_path_distance_navigates_around_obstacles():
    # Straight line distance between (0, 2) and (4, 2) is 4 cells = 2.0m (res=0.5)
    # With a vertical wall at x=2 from y=1 to y=3, path must route around
    wall = {(2, 1), (2, 2), (2, 3)}
    dist = static_grid_path_distance(
        start=(0, 2), goal=(4, 2), blocked=wall, width=10, height=10, resolution=0.5)
    # Bypassing wall: (0,2)->(1,2)->(1,0)->(3,0)->(3,2)->(4,2) = 6 or 8 steps
    assert dist > 2.0  # Must be strictly longer than straight line due to obstacle
    assert dist == pytest.approx(4.0)  # 8 steps * 0.5 = 4.0m


def test_static_grid_path_distance_same_cell():
    assert static_grid_path_distance((5, 5), (5, 5), blocked=set(), width=10, height=10, resolution=0.5) == 0.0


def test_protocol_qos_depth_is_scaled_for_burst_traffic():
    from sih_amr_fleet.common import PROTOCOL_QOS
    assert PROTOCOL_QOS.depth >= 100


def test_random_task_generator_throttling_counts_only_unassigned_tasks():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    assert 'unassigned_count = len([t for t in self.active_tasks if t not in self.executing_tasks])' in generator
    assert 'unassigned_count >= self.max_active_tasks' in generator


def test_cbba_single_task_bundle_capacity_penalty():
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_interfaces.msg import Task
    from sih_amr_fleet.cbba_node import CbbaNode

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        node.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        t2 = Task()
        t2.task_id = 'task_target'
        t2.pickup = Pose2D(x=2.0, y=2.0, theta=0.0)
        t2.dropoff = Pose2D(x=4.0, y=4.0, theta=0.0)

        # Baseline bid without existing claims
        base_bid = node.bid(t2)
        assert base_bid < 1000.0

        # With an existing claim on another task, single-task bundle constraint must apply penalty
        node.busy_robots[node.robot_id] = ('task_claimed', 100.0)
        node.own_claims['task_claimed'] = (node.robot_id, node.session_id, 10.0, 1)
        penalized_bid = node.bid(t2)
        assert penalized_bid >= 1000.0
    finally:
        node.destroy_node()


def test_cbba_unfreezes_bid_penalty_immediately_when_idle():
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_interfaces.msg import Task
    from sih_amr_fleet.cbba_node import CbbaNode

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        node.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        t2 = Task()
        t2.task_id = 'task_avail'
        t2.expires_at.sec = int(node.get_clock().now().nanoseconds / 1e9) + 600
        t2.pickup = Pose2D(x=2.0, y=2.0, theta=0.0)
        t2.dropoff = Pose2D(x=4.0, y=4.0, theta=0.0)
        node.tasks[t2.task_id] = t2

        # Simulate robot had an old busy penalty bid in own_bids while working on another task
        node.own_bids[t2.task_id] = (1015.0, 1)
        assert node.busy_robots.get(node.robot_id) is None  # Robot is now idle

        # run_round must clear penalty and bid true competitive cost immediately
        node.run_round()
        assert t2.task_id in node.own_bids
        assert node.own_bids[t2.task_id][0] < 1000.0
    finally:
        node.destroy_node()


def test_cbba_purges_unconfirmed_phantom_commitments_on_execution_conflict():
    import rclpy
    from sih_amr_interfaces.msg import TaskExecutionStatus
    from sih_amr_fleet.cbba_node import CbbaNode

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        # Simulate an unconfirmed split-brain phantom commitment on robot_3
        node.executing_tasks['t_phantom'] = ('robot_3', 100.0)
        node.committed_claims['t_phantom'] = ('robot_3', 'sess_3', 15.0, 1)
        node.own_claims['t_phantom'] = ('robot_3', 'sess_3', 15.0, 1)

        # Status arrives confirming robot_3 is actually executing a different task
        status = TaskExecutionStatus()
        status.task_id = 't_actual'
        status.owner_robot_id = 'robot_3'
        status.phase = TaskExecutionStatus.EN_ROUTE_PICKUP
        node.on_execution(status)

        # The unconfirmed phantom commitment must be purged immediately
        assert 't_phantom' not in node.executing_tasks
        assert 't_phantom' not in node.committed_claims
        assert 't_phantom' not in node.own_claims
    finally:
        node.destroy_node()


def test_cbba_evicts_stale_peer_claims_differing_from_winner_view():
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_interfaces.msg import Task, TaskConsensus
    from sih_amr_fleet.cbba_node import CbbaNode
    from sih_amr_fleet.common import header

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        node.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        t = Task()
        t.task_id = 't_auction'
        now_s = int(node.get_clock().now().nanoseconds / 1e9)
        t.expires_at.sec = now_s + 600
        node.tasks[t.task_id] = t

        # Peer robot_2 signed a stale claim with an outdated bid of 1015.0
        stale_claim = TaskConsensus()
        stale_claim.fleet_header = header(node, 'robot_2', 'sess_2', 1, 120.0)
        stale_claim.task_id = t.task_id
        stale_claim.winner_robot_id = 'robot_1'
        stale_claim.winner_session_id = node.session_id
        stale_claim.winning_bid = 1015.0
        stale_claim.assignment_epoch = 1
        stale_claim.lease_until = stale_claim.fleet_header.valid_until
        stale_claim.event = TaskConsensus.CLAIM
        node.claim_views[t.task_id] = {'robot_2': stale_claim}

        # Current bids show true cost of 1.0
        node.bid_views[t.task_id] = {
            'robot_1': node.consensus_message(t.task_id, 'robot_1', node.session_id, 1.0, 1, TaskConsensus.BID),
            'robot_2': node.consensus_message(t.task_id, 'robot_2', 'sess_2', 10.0, 1, TaskConsensus.BID),
            'robot_3': node.consensus_message(t.task_id, 'robot_3', 'sess_3', 12.0, 1, TaskConsensus.BID),
            'robot_4': node.consensus_message(t.task_id, 'robot_4', 'sess_4', 14.0, 1, TaskConsensus.BID),
        }
        node.task_seen_at[t.task_id] = now_s - 5.0

        node.run_round()
        # Outdated peer claim must be evicted so it doesn't cause a false lease deadlock
        assert 'robot_2' not in node.claim_views[t.task_id]
    finally:
        node.destroy_node()


def test_cbba_unanimous_commit_purges_unconfirmed_commitments_for_same_winner():
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_interfaces.msg import Task, TaskConsensus
    from sih_amr_fleet.cbba_node import CbbaNode
    from sih_amr_fleet.common import header

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        node.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        # Unconfirmed phantom commitment on t1 for robot_2
        node.executing_tasks['t1'] = ('robot_2', 100.0)
        node.committed_claims['t1'] = ('robot_2', 'sess_2', 15.0, 1)

        t2 = Task()
        t2.task_id = 't2'
        t2.pickup = Pose2D(x=50.0, y=50.0, theta=0.0)
        t2.dropoff = Pose2D(x=50.0, y=50.0, theta=0.0)
        now_s = int(node.get_clock().now().nanoseconds / 1e9)
        t2.expires_at.sec = now_s + 600
        node.tasks['t2'] = t2

        bids = {}
        for r_id, s_id, b in [('robot_1', node.session_id, 30.0), ('robot_2', 'sess_2', 5.0), ('robot_3', 'sess_3', 12.0), ('robot_4', 'sess_4', 14.0)]:
            bm = TaskConsensus()
            bm.fleet_header = header(node, r_id, s_id, 1, 120.0)
            bm.task_id = 't2'
            bm.winner_robot_id = r_id
            bm.winner_session_id = s_id
            bm.winning_bid = b
            bm.assignment_epoch = 1
            bm.lease_until = bm.fleet_header.valid_until
            bm.event = TaskConsensus.BID
            bids[r_id] = bm

        node.bid_views['t2'] = bids
        node.task_seen_at['t2'] = now_s - 5.0

        for r_id, s_id in [('robot_2', 'sess_2'), ('robot_3', 'sess_3'), ('robot_4', 'sess_4')]:
            c = TaskConsensus()
            c.fleet_header = header(node, r_id, s_id, 2, 120.0)
            c.task_id = 't2'
            c.winner_robot_id = 'robot_2'
            c.winner_session_id = 'sess_2'
            c.winning_bid = 5.0
            c.assignment_epoch = 1
            c.lease_until = c.fleet_header.valid_until
            c.event = TaskConsensus.CLAIM
            node.claim_views.setdefault('t2', {})[r_id] = c

        node.run_round()
        assert 't1' not in node.executing_tasks
        assert 't1' not in node.committed_claims
        assert 't2' in node.executing_tasks
        assert node.executing_tasks['t2'][0] == 'robot_2'
    finally:
        node.destroy_node()


def test_cbba_post_completion_rebid_and_allocation():
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_interfaces.msg import Task, TaskConsensus, TaskExecutionStatus
    from sih_amr_fleet.cbba_node import CbbaNode
    from sih_amr_fleet.common import header

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        node.pose = Pose2D(x=1.0, y=1.0, theta=0.0)
        now_s = int(node.get_clock().now().nanoseconds / 1e9)

        # 1. Simulate completion of a prior task
        comp_status = TaskExecutionStatus()
        comp_status.task_id = 't_done'
        comp_status.owner_robot_id = node.robot_id
        comp_status.phase = TaskExecutionStatus.COMPLETED
        node.on_execution(comp_status)

        assert node.robot_id not in node.busy_robots
        assert 't_done' in node.completed_tasks

        # 2. Add a new available task
        t_next = Task()
        t_next.task_id = 't_next'
        t_next.pickup = Pose2D(x=2.0, y=2.0, theta=0.0)
        t_next.dropoff = Pose2D(x=3.0, y=3.0, theta=0.0)
        t_next.expires_at.sec = now_s + 600
        node.tasks['t_next'] = t_next
        node.task_seen_at['t_next'] = now_s - 5.0

        # Robot 1 bid must be clean without busy penalty
        bid_val = node.bid(t_next)
        assert bid_val < 1000.0

        # 3. Populate peer bids where robot 1 is best bidder
        bids = {}
        for r_id, s_id, b in [
            ('robot_1', node.session_id, bid_val),
            ('robot_2', 'sess_2', bid_val + 10.0),
            ('robot_3', 'sess_3', bid_val + 20.0),
            ('robot_4', 'sess_4', bid_val + 30.0)
        ]:
            bm = TaskConsensus()
            bm.fleet_header = header(node, r_id, s_id, 1, 120.0)
            bm.task_id = 't_next'
            bm.winner_robot_id = r_id
            bm.winner_session_id = s_id
            bm.winning_bid = b
            bm.assignment_epoch = 1
            bm.lease_until = bm.fleet_header.valid_until
            bm.event = TaskConsensus.BID
            bids[r_id] = bm
        node.bid_views['t_next'] = bids

        # Populate peer claims matching robot 1
        for r_id, s_id in [('robot_2', 'sess_2'), ('robot_3', 'sess_3'), ('robot_4', 'sess_4')]:
            c = TaskConsensus()
            c.fleet_header = header(node, r_id, s_id, 2, 120.0)
            c.task_id = 't_next'
            c.winner_robot_id = 'robot_1'
            c.winner_session_id = node.session_id
            c.winning_bid = bid_val
            c.assignment_epoch = 1
            c.lease_until = c.fleet_header.valid_until
            c.event = TaskConsensus.CLAIM
            node.claim_views.setdefault('t_next', {})[r_id] = c

        node.run_round()
        assert 't_next' in node.executing_tasks
        assert node.executing_tasks['t_next'][0] == 'robot_1'
    finally:
        node.destroy_node()


def test_cbba_consensus_evicts_unconfirmed_on_differing_peer_winner():
    import rclpy
    from sih_amr_interfaces.msg import TaskConsensus
    from sih_amr_fleet.cbba_node import CbbaNode
    from sih_amr_fleet.common import header

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        # Simulate unconfirmed commitment on t_split for robot_2
        node.executing_tasks['t_split'] = ('robot_2', 100.0)
        node.committed_claims['t_split'] = ('robot_2', 'sess_2', 15.0, 1)

        # Peer robot_3 sends a claim for robot_3
        msg = TaskConsensus()
        msg.fleet_header = header(node, 'robot_3', 'sess_3', 10, 120.0)
        msg.task_id = 't_split'
        msg.winner_robot_id = 'robot_3'
        msg.winner_session_id = 'sess_3'
        msg.winning_bid = 5.0
        msg.assignment_epoch = 1
        msg.lease_until = msg.fleet_header.valid_until
        msg.event = TaskConsensus.CLAIM

        node.on_consensus(msg)
        # Unconfirmed phantom commitment must be evicted rather than dropping the message
        assert 't_split' not in node.executing_tasks
        assert 't_split' not in node.committed_claims
    finally:
        node.destroy_node()


def test_cbba_candidates_evaluates_bids_without_stale_busy_cache_split():
    import rclpy
    from sih_amr_interfaces.msg import TaskConsensus
    from sih_amr_fleet.cbba_node import CbbaNode
    from sih_amr_fleet.common import BUSY_BID_FLOOR, header

    if not rclpy.ok():
        rclpy.init()
    node = CbbaNode()
    try:
        # Simulate node having a stale busy entry for robot_1 from an older task
        node.busy_robots['robot_1'] = ('old_task_001', 9999.0)

        # robot_1 broadcasted an idle bid of 14.25 for new_task
        v1 = TaskConsensus()
        v1.fleet_header = header(node, 'robot_1', 'sess_1', 1, 120.0)
        v1.task_id = 'new_task'
        v1.winner_robot_id = 'robot_1'
        v1.winner_session_id = 'sess_1'
        v1.winning_bid = 14.25
        v1.assignment_epoch = 1

        # robot_2 broadcasted a bid of 19.50 for new_task
        v2 = TaskConsensus()
        v2.fleet_header = header(node, 'robot_2', 'sess_2', 1, 120.0)
        v2.task_id = 'new_task'
        v2.winner_robot_id = 'robot_2'
        v2.winner_session_id = 'sess_2'
        v2.winning_bid = 19.50
        v2.assignment_epoch = 1

        views = {'robot_1': v1, 'robot_2': v2}
        candidates = [
            view for view in views.values()
            if math.isfinite(view.winning_bid) and 0.0 <= view.winning_bid < BUSY_BID_FLOOR
        ]
        winner_view = min(candidates, key=lambda v: (v.winning_bid, v.winner_robot_id))
        # robot_1 must be included in candidates despite local busy_robots cache,
        # preventing 4-vs-2 split brain!
        assert winner_view.winner_robot_id == 'robot_1'
        assert winner_view.winning_bid == 14.25
    finally:
        node.destroy_node()


def test_corridor_mutex_route_requires_mutex_longitudinal_vs_transverse():
    import rclpy
    from sih_amr_interfaces.msg import RoutePlan
    from sih_amr_fleet.corridor_mutex_node import CorridorMutexNode
    from geometry_msgs.msg import Point

    if not rclpy.ok():
        rclpy.init()
    node = CorridorMutexNode()
    try:
        # Create a horizontal corridor NC-TEST with resolution 0.5m spanning x from 10 to 20 at y=5 (5.0m long)
        node.corridors['NC-TEST'] = {(x, 5) for x in range(10, 21)}
        node.corridor_meta['NC-TEST'] = {
            'axis': 'x',
            'min_x': 10, 'max_x': 20,
            'min_y': 5, 'max_y': 5,
        }

        # Case 1: Transverse route crossing across y from (15, 3) to (15, 7)
        # Intersects corridor at only 1 cell: (15, 5). Longitudinal travel = 0m.
        route_cross = RoutePlan()
        route_cross.route_feasible = True
        for y in range(3, 8):
            p = Point()
            p.x = 15.0
            p.y = float(y)
            route_cross.cells.append(p)
        assert not node.route_requires_mutex(route_cross, 'NC-TEST')

        # Case 2: Longitudinal route driving down aisle from (10, 5) to (18, 5)
        # Intersects corridor for 8 cells (4.0m >= 1.5m).
        route_along = RoutePlan()
        route_along.route_feasible = True
        for x in range(10, 19):
            p = Point()
            p.x = float(x)
            p.y = 5.0
            route_along.cells.append(p)
        assert node.route_requires_mutex(route_along, 'NC-TEST')
    finally:
        node.destroy_node()


def test_corridor_approach_cells_only_at_longitudinal_caps():
    import rclpy
    from sih_amr_fleet.corridor_mutex_node import CorridorMutexNode

    if not rclpy.ok():
        rclpy.init()
    node = CorridorMutexNode()
    try:
        package_root = pathlib.Path(__file__).parents[1]
        map_path = package_root.joinpath('maps/demo_warehouse.yaml')
        node.load_map(str(map_path))

        # Check a shelf-row aisle corridor: e.g. NC-NORTH-EAST-01
        corridor = node.corridors.get('NC-NORTH-EAST-01')
        assert corridor is not None
        app = node.approach_cells.get('NC-NORTH-EAST-01', set())
        assert len(app) > 0

        # All cells in NC-NORTH-EAST-01 share the same cy
        cy = next(iter(corridor))[1]
        # In the new longitudinal entrance caps design, approach cells share the same cy
        # (they extend west and east of the aisle endpoints), but NOT cy+1 or cy-1 (adjacent shelves/aisles)
        for ax, ay in app:
            assert ay == cy, f"Approach cell ({ax}, {ay}) spilled into adjacent y levels (expected cy={cy})"
    finally:
        node.destroy_node()


def test_avoidance_stationary_peer_stop():
    """Verify that a moving AMR yields 100% and brakes before a stationary peer."""
    preferred = (0.0, -0.46)
    self_xy = (-6.0, -9.0)
    stationary_peer = {
        'x': -6.0, 'y': -10.0,
        'vx': 0.0, 'vy': 0.0,
        'radius_inflation': 0.0,
        'id': 'robot_4'
    }
    vx, vy = avoidance_velocity(preferred, self_xy, [stationary_peer], radius=0.35, horizon=1.5, max_speed=0.46, self_id='robot_3')
    # Forward velocity must be heavily braked (< 0.40 m/s magnitude) regardless of robot ID comparison
    assert abs(vy) < 0.40, f"Expected braking for stationary peer, got vy={vy:.3f}"


def test_avoidance_reciprocal_moving_peers():
    """Verify that two moving peers on a collision course reciprocally decelerate."""
    # Robot 3 moving south at 0.46 m/s
    r3_xy = (-6.0, -9.0)
    r3_preferred = (0.0, -0.46)

    # Robot 4 moving north at 0.46 m/s
    r4_peer = {
        'x': -6.0, 'y': -11.0,
        'vx': 0.0, 'vy': 0.46,
        'radius_inflation': 0.0,
        'id': 'robot_4'
    }

    v3_x, v3_y = avoidance_velocity(r3_preferred, r3_xy, [r4_peer], radius=0.35, horizon=1.5, max_speed=0.46, self_id='robot_3')

    # Reverse perspective: Robot 4 observing Robot 3
    r4_xy = (-6.0, -11.0)
    r4_preferred = (0.0, 0.46)
    r3_peer = {
        'x': -6.0, 'y': -9.0,
        'vx': 0.0, 'vy': -0.46,
        'radius_inflation': 0.0,
        'id': 'robot_3'
    }

    v4_x, v4_y = avoidance_velocity(r4_preferred, r4_xy, [r3_peer], radius=0.35, horizon=1.5, max_speed=0.46, self_id='robot_4')

    # Both robots must reciprocally brake (|vy| < 0.46 m/s) with balanced magnitudes
    assert abs(v3_y) < 0.46, f"Robot 3 should brake, got {v3_y}"
    assert abs(v4_y) < 0.46, f"Robot 4 should brake, got {v4_y}"
    assert abs(abs(v3_y) - abs(v4_y)) < 1e-4, f"Reciprocal braking must be symmetric: v3_y={v3_y}, v4_y={v4_y}"


def test_outer_west_wall_column_is_blocked():
    """Verify that outer perimeter column cx=0 is blocked across all height rows."""
    raw_data = {
        'grid': {'resolution': 0.5, 'width': 90, 'height': 120, 'origin_x': -22.5, 'origin_y': -30.0},
        'shelves': [],
        'charging_pads': [],
    }
    resolution, width, height, origin_x, origin_y, blocked = map_geometry_from_data(raw_data)
    for cy in range(height):
        assert (0, cy) in blocked, f"Boundary cell (0, {cy}) must be blocked to prevent wall gap entry"


def test_western_shelf_path_exits_east_never_west_wall():
    """Verify that a path from a western shelf exits east towards main corridor and never visits cx=0."""
    layout_path = pathlib.Path(__file__).parents[3] / 'warehouse_layout.lock.yaml'
    if not layout_path.exists():
        layout_path = pathlib.Path('warehouse_layout.lock.yaml')
    with open(layout_path, 'r') as f:
        layout = yaml.safe_load(f)
    resolution, width, height, origin_x, origin_y, blocked = map_geometry_from_data(layout)
    # Start at western shelf aisle cell (1, 25), goal at main corridor cell (30, 25)
    start = (1, 25)
    goal = (30, 25)
    path = whca_star(
        start=start,
        goal=goal,
        blocked=blocked,
        reservations=set(),
        width=width,
        height=height,
        horizon=100
    )
    assert path, "Path from western shelf must be reachable"
    for cx, cy, _ in path:
        assert cx > 0, f"Path visited outer west wall cell ({cx}, {cy})"
    # Path must progress eastward (cx increasing) towards the main corridor (cx=30)
    assert path[-1][0] == 30


def test_safety_supervisor_node_enforce():
    """Verify that SafetySupervisorNode.enforce() executes cleanly without NameError."""
    import rclpy
    from sih_amr_fleet.safety_supervisor_node import SafetySupervisorNode

    if not rclpy.ok():
        rclpy.init()
    node = SafetySupervisorNode()
    try:
        # Candidate stopped
        node.candidate.linear.x = 0.0
        node.candidate.angular.z = 0.0
        node.enforce()

        # Candidate moving forward at tracking speed
        node.candidate.linear.x = 0.46
        node.candidate.angular.z = 0.1
        node.pose_time = node.get_clock().now().nanoseconds / 1e9
        node.scan_time = node.get_clock().now().nanoseconds / 1e9
        node.enforce()

        # Candidate moving backward
        node.candidate.linear.x = -0.2
        node.candidate.angular.z = 0.0
        node.enforce()
    finally:
        node.destroy_node()


def test_corridor_mutex_throat_sweep_protection():
    """Verify that CorridorMutexNode protects the junction throat sweep until AMR clears the mouth."""
    import rclpy
    from sih_amr_fleet.corridor_mutex_node import CorridorMutexNode
    from sih_amr_interfaces.msg import RobotState
    from geometry_msgs.msg import Pose2D

    if not rclpy.ok():
        rclpy.init()
    node = CorridorMutexNode()
    try:
        node.origin_x = -22.5
        node.origin_y = -30.0
        node.resolution = 0.5
        node.corridors['NC-TEST'] = {(x, 50) for x in range(60, 89)}
        node.corridor_meta['NC-TEST'] = {
            'axis': 'x',
            'min_x': 60, 'max_x': 88,
            'min_y': 50, 'max_y': 50,
        }

        # Simulate entered and inside corridor
        node.request = {
            'corridor': 'NC-TEST',
            'id': 'req-test-1',
            'ts': 1,
            'grants': {'robot_2'},
            'entered': True,
            'was_inside': True,
            'created_at': node.get_clock().now().nanoseconds * 1e-9,
        }

        state = RobotState()
        state.fleet_header.robot_id = 'robot_1'

        # Case 1: AMR is inside aisle at x=10.0 (cell 65), y=-5.0 (cell 50)
        state.pose = Pose2D(x=10.0, y=-5.0, theta=3.14)
        node.on_state(state)
        assert node.request is not None, "AMR inside aisle must retain mutex"

        # Case 2: AMR moves into junction throat at x=6.5 (cell 58, 2 cells away from min_x 60)
        # y=-5.0 (still aligned with aisle centreline). Under the old logic this would release mutex!
        state.pose = Pose2D(x=6.5, y=-5.0, theta=3.14)
        node.on_state(state)
        assert node.request is not None, "AMR in throat sweep must retain mutex until turning or clearing"

        # Case 3: AMR turns North into corridor: x=6.5, y=-3.5 (dist_y = 1.5m >= 0.8m)
        state.pose = Pose2D(x=6.5, y=-3.5, theta=1.57)
        node.on_state(state)
        assert node.request is None, "AMR that turned into cross corridor must release mutex with EXIT"
    finally:
        node.destroy_node()


def test_whca_stationary_peer_4d_reservation():
    """Verify that WhcaPlannerNode routes around a stationary peer via 4D reservations without static 2D blockage."""
    import rclpy
    from geometry_msgs.msg import Pose2D, Twist
    from sih_amr_fleet.whca_planner_node import WhcaPlannerNode
    from sih_amr_interfaces.msg import FleetHeader, RobotState, Task, TaskAssignment

    if not rclpy.ok():
        rclpy.init()
    node = WhcaPlannerNode()
    try:
        layout_path = pathlib.Path(__file__).parents[3] / 'warehouse_layout.lock.yaml'
        if not layout_path.exists():
            layout_path = pathlib.Path('warehouse_layout.lock.yaml')
        node.load_map(str(layout_path))
        node.robot_id = 'robot_2'
        node.pose = Pose2D(x=6.0, y=-1.5, theta=-1.57)

        assignment = TaskAssignment()
        assignment.task = Task()
        assignment.task.task_id = 'test_avoid_4d'
        assignment.task.pickup = Pose2D(x=6.0, y=-7.5, theta=0.0)
        node.assignment = assignment
        node.execution_task_id = 'test_avoid_4d'

        # Peer (robot_4) is stationary directly ahead at (6.0, -4.0) [cell (57, 52)]
        peer_state = RobotState()
        peer_state.fleet_header = FleetHeader(robot_id='robot_4')
        peer_state.pose = Pose2D(x=6.0, y=-4.0, theta=1.57)
        peer_state.twist = Twist()
        peer_state.localization_valid = True

        node.on_state(peer_state)
        now = node.get_clock().now().nanoseconds * 1e-9
        node.peer_tracking['robot_4']['stopped_since'] = now - 2.5
        node.peer_tracking['robot_4']['last_seen'] = now

        node.plan()
        assert node.plan_id > 0, "Plan should be generated"
        # Peer cell (57, 52) must be in reservations for t in [0, 12]
        peer_cell = node.to_cell(peer_state.pose)
        # Static blocked should NOT contain peer cell (graph connectivity preserved)
        assert peer_cell not in node.static_blocked
    finally:
        node.destroy_node()


def test_path_follower_orca_stall_detection_and_yield():
    """Verify that PathFollowerNode detects ORCA stalls and breaks symmetry based on priority."""
    import rclpy
    from geometry_msgs.msg import Point, Pose2D
    from sih_amr_fleet.path_follower_node import PathFollowerNode
    from sih_amr_interfaces.msg import RoutePlan

    if not rclpy.ok():
        rclpy.init()

    # Robot 4: lower priority than robot_2 -> should initiate recovery
    node4 = PathFollowerNode()
    try:
        node4.robot_id = 'robot_4'
        node4.pose = Pose2D(x=6.0, y=-5.0, theta=1.57)
        node4.reverse_clearance = 4.0

        route = RoutePlan()
        route.route_feasible = True
        for y in (-4.0, -3.0, -2.0):
            p = Point()
            p.x = 6.0
            p.y = float(y)
            route.waypoints.append(p)
        node4.route = route

        now = node4.get_clock().now().nanoseconds * 1e-9
        node4.peers['robot_2'] = {
            'pose': Pose2D(x=6.0, y=-3.5, theta=-1.57),
            'last_seen': now,
        }
        node4.measured_speed = 0.0
        node4.nearest = 1.5

        # First tick: starts stall timer
        node4.control()
        assert node4.stall_started is not None
        assert node4.recovery_state == 'IDLE'

        # Fast forward time by 3 seconds
        node4.stall_started = now - 3.0
        node4.control()
        # Robot 4 yields (robot_2 < robot_4) -> transitions to VERIFY
        assert node4.recovery_state == 'VERIFY'
    finally:
        node4.destroy_node()

    # Robot 2: higher priority than robot_4 -> should NOT yield
    node2 = PathFollowerNode()
    try:
        node2.robot_id = 'robot_2'
        node2.pose = Pose2D(x=6.0, y=-3.5, theta=-1.57)
        node2.reverse_clearance = 4.0

        route2 = RoutePlan()
        route2.route_feasible = True
        for y in (-4.5, -5.5, -6.5):
            p = Point()
            p.x = 6.0
            p.y = float(y)
            route2.waypoints.append(p)
        node2.route = route2

        now = node2.get_clock().now().nanoseconds * 1e-9
        node2.peers['robot_4'] = {
            'pose': Pose2D(x=6.0, y=-5.0, theta=1.57),
            'last_seen': now,
        }
        node2.measured_speed = 0.0
        node2.nearest = 1.5

        node2.control()
        node2.stall_started = now - 3.0
        node2.control()
        # Robot 2 does NOT yield (robot_4 is not < robot_2) -> stays IDLE
        assert node2.recovery_state == 'IDLE'
    finally:
        node2.destroy_node()


def test_whca_reservation_buffer_bypass():
    """Verify that WHCA* with reservation_buffer_cells=3 steps laterally around a stationary peer in a wide corridor."""
    from sih_amr_fleet.algorithms import whca_star

    start = (33, 10)
    goal = (33, 0)
    peer = (33, 5)
    reservations = set((peer[0], peer[1], t) for t in range(12))

    path = whca_star(start, goal, blocked=set(), reservations=reservations,
                     width=90, height=120, horizon=12, reservation_buffer_cells=3, heading_rad=-1.57)
    assert len(path) > 0, "Path must be feasible"
    # AMR must step laterally (nx != 33) to maintain buffer from peer at (33, 5)
    lateral_steps = [p for p in path if p[0] != 33]
    assert len(lateral_steps) > 0, "WHCA* must plan a lateral bypass around the peer"
    # Minimum Chebyshev distance to peer at time matching peer reservation must be > 0
    for p in path:
        dist_to_peer = max(abs(p[0] - peer[0]), abs(p[1] - peer[1]))
        assert dist_to_peer >= 2, f"Path at t={p[2]} violates safety separation: dist={dist_to_peer}"


def test_whca_adaptive_escape_when_inside_buffer():
    """Verify that an AMR starting inside a peer's reservation buffer escapes sideways/away rather than failing."""
    from sih_amr_fleet.algorithms import whca_star

    start = (33, 7)  # Distance 2 from peer at (33, 5) while buffer=3
    goal = (50, 7)
    peer = (33, 5)
    reservations = set((peer[0], peer[1], t) for t in range(12))

    path = whca_star(start, goal, blocked=set(), reservations=reservations,
                     width=90, height=120, horizon=12, reservation_buffer_cells=3, heading_rad=-1.57)
    assert len(path) > 0, "AMR inside buffer must be allowed to escape"
    # Step at t=1 must NOT move closer to peer
    p1 = path[1]
    d1 = max(abs(p1[0] - peer[0]), abs(p1[1] - peer[1]))
    assert d1 >= 2, f"AMR must not move closer to peer: d1={d1}"


def test_path_follower_retreat_extended_timeout_and_stand_down():
    """Verify that path follower handles extended timeout, adaptive completion, and stand-down."""
    import rclpy
    from sih_amr_fleet.path_follower_node import PathFollowerNode
    from geometry_msgs.msg import Pose2D

    if not rclpy.ok():
        rclpy.init()
    node = PathFollowerNode()
    try:
        node.pose = Pose2D(x=6.0, y=0.0, theta=1.57)
        node.recovery_reverse_m = 2.0
        node.recovery_speed_mps = 0.20

        # Initiate REVERSING
        node.recovery_state = 'REVERSING'
        node.recovery_start_pose = (6.0, 0.0)
        node.recovery_started = 100.0

        # At t=105.0s (5s in), distance reversed = 1.2m, peer_dist = 3.0m (clear)
        node.pose.y = -1.2
        # Mock peer at y = 1.8 (distance = 3.0m)
        node.peers['robot_1'] = {
            'pose': Pose2D(x=6.0, y=1.8, theta=-1.57),
            'last_seen': 105.0,
            'speed': 0.46
        }
        # Simulate check in control:
        distance = math.hypot(node.pose.x - node.recovery_start_pose[0], node.pose.y - node.recovery_start_pose[1])
        peer_dist = math.hypot(node.peers['robot_1']['pose'].x - node.pose.x, node.peers['robot_1']['pose'].y - node.pose.y)
        assert distance >= 1.0
        assert peer_dist >= 2.5
    finally:
        node.destroy_node()


def test_path_follower_stand_down_adaptive_peer_clearing():
    """Verify that STAND_DOWN holds while peer is ahead and releases once peer clears."""
    import rclpy
    from sih_amr_fleet.path_follower_node import PathFollowerNode
    from geometry_msgs.msg import Pose2D

    if not rclpy.ok():
        rclpy.init()
    node = PathFollowerNode()
    try:
        node.pose = Pose2D(x=-6.0, y=-8.0, theta=1.57) # Facing North (+y)
        node.recovery_state = 'STAND_DOWN'
        node.recovery_started = 100.0
        node.recovery_stand_down_until = 108.0

        # Peer is at (-6.0, -6.0) (2.0m ahead along track) -> NOT cleared
        node.peers['robot_1'] = {
            'pose': Pose2D(x=-6.0, y=-6.0, theta=-1.57),
            'last_seen': 102.0,
            'speed': 0.46
        }
        now = 102.0
        dx = node.peers['robot_1']['pose'].x - node.pose.x
        dy = node.peers['robot_1']['pose'].y - node.pose.y
        dist = math.hypot(dx, dy)
        along = dx * math.cos(node.pose.theta) + dy * math.sin(node.pose.theta)
        assert along > -0.2 and dist < 3.2, "Peer is directly ahead, must not be cleared"

        # Peer moves past to (-6.0, -9.0) (behind us: along = -1.0m) -> Cleared!
        node.peers['robot_1']['pose'] = Pose2D(x=-6.0, y=-9.0, theta=-1.57)
        dx = node.peers['robot_1']['pose'].x - node.pose.x
        dy = node.peers['robot_1']['pose'].y - node.pose.y
        dist = math.hypot(dx, dy)
        along = dx * math.cos(node.pose.theta) + dy * math.sin(node.pose.theta)
        assert along <= -0.2 or dist >= 3.2, "Peer is behind, must be cleared"
    finally:
        node.destroy_node()


def test_whca_planner_destination_inside_mutex_corridor():
    """Verify that a goal inside a mutex corridor plans with route_feasible=True."""
    import rclpy
    from geometry_msgs.msg import Pose2D
    from sih_amr_fleet.whca_planner_node import WhcaPlannerNode
    from sih_amr_interfaces.msg import Task, TaskAssignment

    if not rclpy.ok():
        rclpy.init()
    node = WhcaPlannerNode()
    try:
        layout_path = pathlib.Path(__file__).parents[3] / 'warehouse_layout.lock.yaml'
        if not layout_path.exists():
            layout_path = pathlib.Path('warehouse_layout.lock.yaml')
        node.load_map(str(layout_path))
        node.robot_id = 'robot_2'
        node.pose = Pose2D(x=-6.0, y=-8.0, theta=1.57)

        # Peer robot_1 occupies narrow_junction_middle_centre
        node.occupied_corridors['robot_1'] = 'narrow_junction_middle_centre'

        # Goal is rnd_task_004 pickup at (0.0, 3.0525) inside narrow_junction_middle_centre
        assignment = TaskAssignment()
        assignment.task = Task()
        assignment.task.task_id = 'rnd_task_004'
        assignment.task.pickup = Pose2D(x=0.0, y=3.0525, theta=0.0)
        node.assignment = assignment
        node.execution_task_id = 'rnd_task_004'

        # Verify that clear_goal=True leaves goal free
        goal_cell = node.to_cell(assignment.task.pickup)
        occupied_corridor_cells = set()
        for peer_id, corridor_name in node.occupied_corridors.items():
            occupied_corridor_cells.update(node.corridors.get(corridor_name, set()))

        from sih_amr_fleet.algorithms import clear_nearfield_blockages, whca_star
        planning_blockages = clear_nearfield_blockages(
            set(node.blockages) | occupied_corridor_cells, (33, 44), goal_cell,
            clear_goal=True,
            clearance_cells=1,
        )
        assert goal_cell not in planning_blockages, "Goal cell must never be blocked"

        path = whca_star(
            (33, 44), goal_cell, node.static_blocked | planning_blockages, set(),
            node.width, node.height, node.horizon, node.reservation_buffer_cells
        )
        assert bool(path), "Path to goal inside corridor must be feasible"
    finally:
        node.destroy_node()











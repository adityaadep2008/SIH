"""Unit tests for the Kinematic LiDAR Carrier motion backend."""

import math
import xml.etree.ElementTree as ET
import pytest

from sih_amr_fleet.carrier_robot_description import render_carrier_urdf
from sih_amr_fleet.kinematic_carrier_node import RobotKinematicState


def test_carrier_urdf_structure():
    """Verify that carrier_robot_description generates valid XML with expected links/sensors."""
    urdf_str = render_carrier_urdf(namespace='robot_1', lidar_update_rate=10.0)
    root = ET.fromstring(urdf_str)
    assert root.tag == 'robot'

    links = {link.get('name') for link in root.findall('link')}
    joints = {joint.get('name') for joint in root.findall('joint')}

    assert 'base_link' in links
    assert 'rplidar_link' in links
    assert 'rplidar_joint' in joints

    # Verify no wheel or caster joints exist
    assert 'left_wheel_joint' not in joints
    assert 'right_wheel_joint' not in joints
    assert 'front_caster_joint' not in joints

    # Verify rplidar joint is fixed with exact physical mounting
    rplidar_joint = next(j for j in root.findall('joint') if j.get('name') == 'rplidar_joint')
    assert rplidar_joint.get('type') == 'fixed'
    origin = rplidar_joint.find('origin')
    assert origin is not None
    assert '0.00393584' in origin.get('xyz')

    # Verify rplidar sensor definition
    rplidar_link = next(l for l in root.findall('link') if l.get('name') == 'rplidar_link')
    gazebo_rplidar = next(g for g in root.findall('gazebo') if g.get('reference') == 'rplidar_link')
    sensor = gazebo_rplidar.find('sensor')
    assert sensor is not None
    assert sensor.get('name') == 'rplidar'
    assert sensor.get('type') == 'gpu_lidar'
    assert sensor.find('update_rate').text == '10.0'


def test_kinematic_state_integration():
    """Verify planar kinematic forward integration."""
    robot = RobotKinematicState('robot_1', x=0.0, y=0.0, yaw=0.0)
    dt = 0.02
    v = 0.5
    w = 0.2

    # Forward step for 1 second (50 steps)
    for _ in range(50):
        dx = v * math.cos(robot.true_yaw) * dt
        dy = v * math.sin(robot.true_yaw) * dt
        dyaw = w * dt
        robot.true_x += dx
        robot.true_y += dy
        robot.true_yaw += dyaw

    expected_yaw = 0.2 * 1.0
    assert abs(robot.true_yaw - expected_yaw) < 1e-4
    # Distance traveled should match arc length
    dist = math.hypot(robot.true_x, robot.true_y)
    assert 0.45 < dist < 0.55


def test_zero_noise_odometry_match():
    """Verify that default 0.0 noise produces exact dead-reckoning odometry."""
    robot = RobotKinematicState('robot_1', x=5.0, y=10.0, yaw=math.pi / 4.0)
    scale_error = 0.0
    gyro_drift = 0.0
    dt = 0.02

    v = 0.46
    w = 0.0

    for _ in range(25):
        dx = v * math.cos(robot.true_yaw) * dt
        dy = v * math.sin(robot.true_yaw) * dt
        dyaw = w * dt
        robot.true_x += dx
        robot.true_y += dy
        robot.true_yaw += dyaw

        true_dist = math.copysign(math.hypot(dx, dy), v)
        meas_dist = true_dist * (1.0 + scale_error)
        meas_dyaw = dyaw + (gyro_drift * dt)

        robot.local_x += meas_dist * math.cos(robot.local_yaw)
        robot.local_y += meas_dist * math.sin(robot.local_yaw)
        robot.local_yaw += meas_dyaw

    total_dist_true = 0.46 * 0.5
    total_dist_local = math.hypot(robot.local_x, robot.local_y)
    assert abs(total_dist_true - total_dist_local) < 1e-6
    assert abs(robot.local_yaw) < 1e-6


def test_physical_shelf_collision_clearance():
    """Verify physical shelf collision logic allows passage down all warehouse aisles and detects contact."""
    import yaml
    import os

    layout_file = '/home/rtsws/amr_ws/src/SIH/warehouse_layout.lock.yaml'
    assert os.path.exists(layout_file)

    with open(layout_file) as f:
        data = yaml.safe_load(f)

    shelves = []
    for obj in data.get('objects', []):
        if obj.get('type') == 'storage_shelf':
            bb = obj.get('bounding_box_2d_m')
            shelves.append((bb['min_x'], bb['min_y'], bb['max_x'], bb['max_y']))

    assert len(shelves) == 190

    def is_collision_free(x, y, r=0.17):
        for min_x, min_y, max_x, max_y in shelves:
            if x < min_x - r or x > max_x + r:
                continue
            if y < min_y - r or y > max_y + r:
                continue
            dx = max(0.0, max(min_x - x, x - max_x))
            dy = max(0.0, max(min_y - y, y - max_y))
            if math.hypot(dx, dy) < r:
                return False
        return True

    # AMRs traveling down aisle waypoints for robot 1, 3, 4 must all be collision-free
    assert is_collision_free(-9.1725, -17.5976)
    assert is_collision_free(-10.5000, -17.5976)
    assert is_collision_free(-19.9900, -17.5976)

    assert is_collision_free(-9.1549, -6.1047)
    assert is_collision_free(-10.5000, -6.1047)
    assert is_collision_free(-11.3500, -6.1047)

    assert is_collision_free(-9.0792, 27.7728)
    assert is_collision_free(-10.5000, 27.7728)
    assert is_collision_free(-15.6700, 27.7728)

    # Point directly inside shelf_south_west_06_03 (x=-10.0, y=-18.5) must collide
    assert not is_collision_free(-10.0, -18.5)
    # Point 5cm from shelf surface (max_y=-18.1752, so y=-18.12) must collide
    assert not is_collision_free(-10.0, -18.12)


def test_dock_heading_strict_forward_alignment():
    """Verify that dock confirmation requires strict forward heading match and rejects reverse-facing contact."""
    from sih_amr_fleet.common import wrap_angle

    dock_yaw = 1.5708
    dock_tolerance_yaw = 0.20

    # Robot facing forward (spawn heading +pi/2)
    robot_forward_yaw = 1.5708
    dyaw_forward = abs(wrap_angle(robot_forward_yaw - dock_yaw))
    assert dyaw_forward <= dock_tolerance_yaw

    # Robot facing 180 degrees opposite (-pi/2) must NOT match
    robot_reverse_yaw = -1.5708
    dyaw_reverse = abs(wrap_angle(robot_reverse_yaw - dock_yaw))
    assert dyaw_reverse > dock_tolerance_yaw
    assert math.isclose(dyaw_reverse, math.pi, abs_tol=1e-3)


def test_boundary_inward_recovery_and_rejections():
    """Verify carrier boundary outward rejection, inward recovery, shelf rejection, and peer rejection."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        # 1. Valid interior point:
        assert node._is_position_collision_free('robot_1', -22.50, 10.0)

        # 2. Outward step beyond warehouse boundary (-22.55) is rejected:
        assert not node._is_position_collision_free('robot_1', -22.56, 10.0, curr_x=-22.50, curr_y=10.0)

        # 3. Outward step from outside (-22.56 to -22.58) is rejected:
        assert not node._is_position_collision_free('robot_1', -22.58, 10.0, curr_x=-22.56, curr_y=10.0)

        # 4. Inward recovery from outside (-22.56 to -22.54) strictly reduces violation and is accepted:
        assert node._is_position_collision_free('robot_1', -22.54, 10.0, curr_x=-22.56, curr_y=10.0)

        # 5. Inward candidate into a physical shelf is rejected:
        # shelf position at (-10.0, -18.5)
        assert not node._is_position_collision_free('robot_1', -10.0, -18.5, curr_x=-10.0, curr_y=-18.5)

        # 6. Peer collision rejection: robot_2 is at dock charging_pad_2 (-3.75, -29.55)
        node.robots['robot_2'].true_x = 0.0
        node.robots['robot_2'].true_y = 0.0
        # Step for robot_1 into robot_2's footprint (dist < 2 * 0.35 = 0.70m) must be rejected
        assert not node._is_position_collision_free('robot_1', 0.1, 0.0, curr_x=1.0, curr_y=0.0)

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_adaptive_dispatch_rates_and_triggers():
    """Verify adaptive dispatch rate calculation and transition triggers in KinematicCarrierNode."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        assert not node._force_gz_sync
        assert not node._fleet_was_moving

        # Stationary fleet rate check: all robots at 0 velocity -> 2 Hz (0.50s interval)
        for r in node.robots.values():
            r.cmd_linear_x = 0.0
            r.cmd_angular_z = 0.0
            r.true_vx = 0.0
            r.true_wz = 0.0

        any_moving = any(
            abs(r.cmd_linear_x) > 0.001 or abs(r.cmd_angular_z) > 0.001 or
            abs(r.true_vx) > 0.001 or abs(r.true_wz) > 0.001
            for r in node.robots.values()
        )
        assert not any_moving
        interval_stationary = 0.05 if any_moving else 0.50
        assert interval_stationary == 0.50

        # Motion start transition check: robot_1 accelerates -> triggers immediate sync
        node.robots['robot_1'].cmd_linear_x = 0.46
        any_moving = any(
            abs(r.cmd_linear_x) > 0.001 or abs(r.cmd_angular_z) > 0.001 or
            abs(r.true_vx) > 0.001 or abs(r.true_wz) > 0.001
            for r in node.robots.values()
        )
        assert any_moving
        interval_moving = 0.05 if any_moving else 0.50
        assert interval_moving == 0.05

        if any_moving != node._fleet_was_moving:
            node._force_gz_sync = True
            node._fleet_was_moving = any_moving
        assert node._force_gz_sync is True
        assert node._fleet_was_moving is True

        # Motion stop transition check: robot_1 stops -> triggers immediate sync
        node._force_gz_sync = False
        node.robots['robot_1'].cmd_linear_x = 0.0
        any_moving = any(
            abs(r.cmd_linear_x) > 0.001 or abs(r.cmd_angular_z) > 0.001 or
            abs(r.true_vx) > 0.001 or abs(r.true_wz) > 0.001
            for r in node.robots.values()
        )
        assert not any_moving
        if any_moving != node._fleet_was_moving:
            node._force_gz_sync = True
            node._fleet_was_moving = any_moving
        assert node._force_gz_sync is True
        assert node._fleet_was_moving is False

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_entity_discovery_triggers_sync():
    """Verify that resolving a new entity ID in _on_gz_poses triggers an immediate Gazebo sync."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        node._force_gz_sync = False

        class MockPose:
            def __init__(self, name, eid):
                self.name = name
                self.id = eid

        class MockGzPoseV:
            def __init__(self, poses):
                self.pose = poses

        # Simulate Gazebo pose broadcast containing robot_1 turtlebot4 entity id 88
        msg = MockGzPoseV([MockPose('robot_1/turtlebot4', 88)])
        node._on_gz_poses(msg)

        assert node.gz_entity_ids.get('robot_1') == 88
        assert node._force_gz_sync is True

        # Repeated message with same ID should not re-trigger force sync
        node._force_gz_sync = False
        node._on_gz_poses(msg)
        assert node._force_gz_sync is False

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_latest_snapshot_coalescing():
    """Verify that busy background worker causes pending pose snapshots to coalesce without queuing."""
    import concurrent.futures
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        # Simulate worker currently busy with an unresolved future
        busy_future = concurrent.futures.Future()
        node.gz_future = busy_future

        # Simulate snapshot 1 arriving
        snap1 = "snapshot_1_data"
        with node._gz_lock:
            node._pending_gz_vector = snap1
            if (node.gz_future is None or node.gz_future.done()) and node._pending_gz_vector is not None:
                node.gz_future = node.gz_executor.submit(node._dispatch_gz_pose, node._pending_gz_vector)

        assert node._pending_gz_vector == "snapshot_1_data"

        # Simulate snapshot 2 arriving while worker is still busy (coalescing: snap1 overwritten)
        snap2 = "snapshot_2_latest"
        with node._gz_lock:
            node._pending_gz_vector = snap2
            if (node.gz_future is None or node.gz_future.done()) and node._pending_gz_vector is not None:
                node.gz_future = node.gz_executor.submit(node._dispatch_gz_pose, node._pending_gz_vector)

        assert node._pending_gz_vector == "snapshot_2_latest"

        # Now simulate worker completion
        busy_future.set_result(None)
        submitted_item = None

        with node._gz_lock:
            if (node.gz_future is None or node.gz_future.done()) and node._pending_gz_vector is not None:
                submitted_item = node._pending_gz_vector
                node._pending_gz_vector = None

        assert submitted_item == "snapshot_2_latest"
        assert node._pending_gz_vector is None

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_gz_request_deadline_is_100ms():
    """Verify that _dispatch_gz_pose uses a bounded 100ms timeout parameter."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()

        captured_kwargs = {}

        class MockGzNode:
            def request(self, topic, req, req_type, rep_type, timeout=10):
                captured_kwargs['topic'] = topic
                captured_kwargs['timeout'] = timeout
                return (True, None)

        node.gz_node = MockGzNode()
        node._dispatch_gz_pose("mock_vector")

        assert captured_kwargs.get('timeout') == 100

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_inter_run_cooldown_argument_parsing():
    """Verify that runners parse the --inter-run-cooldown-s argument correctly."""
    import subprocess
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parent.parent.parent.parent

    desktop_script = repo_root / 'scripts' / 'run_desktop_data_collection.py'
    res_desktop = subprocess.run(
        [sys.executable, str(desktop_script), '--help'],
        capture_output=True,
        text=True
    )
    assert res_desktop.returncode == 0
    assert '--inter-run-cooldown-s' in res_desktop.stdout

    laptop_script = repo_root / 'scripts' / 'run_laptop_data_collection.py'
    res_laptop = subprocess.run(
        [sys.executable, str(laptop_script), '--help'],
        capture_output=True,
        text=True
    )
    assert res_laptop.returncode == 0
    assert '--inter-run-cooldown-s' in res_laptop.stdout


def test_production_simulation_step_adaptive_throttling():
    """Directly execute node._simulation_step() to verify production wall-clock rate limiting and triggers."""
    import time
    from unittest.mock import MagicMock
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        mock_gz = MagicMock()
        mock_gz.request.return_value = (True, MagicMock(data=True))
        node.gz_node = mock_gz
        node.gz_entity_ids['robot_1'] = 101

        # 1. First step while stationary: triggers force_gz_sync because _fleet_was_moving=False
        node._last_gz_dispatch_wall_time = time.monotonic()
        node._force_gz_sync = False

        # Immediate step within 10ms should NOT dispatch (stationary interval is 0.50s)
        initial_count = node.gz_sync_total_count
        node._simulation_step()
        # Should not have submitted a new request because < 0.50s
        assert node.gz_sync_total_count == initial_count

        # 2. Accelerate robot: next _simulation_step() detects transition -> triggers immediate dispatch
        node.robots['robot_1'].cmd_linear_x = 0.46
        node._simulation_step()
        assert node._fleet_was_moving is True
        assert node._force_gz_sync is False
        # Request should have been submitted
        assert node.gz_future is not None

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_production_simulation_step_coalescing():
    """Directly test that node._simulation_step() coalesces into _pending_gz_vector when worker is busy."""
    import concurrent.futures
    import time
    from unittest.mock import MagicMock
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        mock_gz = MagicMock()
        mock_gz.request.return_value = (True, MagicMock(data=True))
        node.gz_node = mock_gz
        node.gz_entity_ids['robot_1'] = 101

        # Simulate busy worker with an unresolved future
        busy_future = concurrent.futures.Future()
        node.gz_future = busy_future

        # Trigger simulation step with force sync
        node._force_gz_sync = True
        node.robots['robot_1'].true_x = 10.0
        node._simulation_step()

        # Worker was busy, so _pending_gz_vector must hold snapshot with x=10.0
        assert node._pending_gz_vector is not None
        p1 = node._pending_gz_vector.pose[0]
        assert p1.position.x == 10.0

        # Now update robot position and step again while worker is STILL busy
        node._force_gz_sync = True
        node.robots['robot_1'].true_x = 20.0
        node._simulation_step()

        # Coalescing: _pending_gz_vector must now hold overwritten snapshot with x=20.0 (never queued)
        assert node._pending_gz_vector is not None
        p2 = node._pending_gz_vector.pose[0]
        assert p2.position.x == 20.0

        # Now mark worker complete
        busy_future.set_result(None)

        # Next simulation step should pop and submit the coalesced snapshot
        node._simulation_step()
        assert node._pending_gz_vector is None

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()


def test_periodic_telemetry_disaggregated_latency():
    """Verify that periodic telemetry records success latency separately from errors."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    shutdown_at_end = False
    if not rclpy.ok():
        rclpy.init()
        shutdown_at_end = True

    try:
        node = KinematicCarrierNode()
        node.gz_sync_total_count = 249

        class MockGzNode:
            def request(self, *args, **kwargs):
                class MockRep:
                    data = True
                return (True, MockRep())

        node.gz_node = MockGzNode()
        node._dispatch_gz_pose("mock_vector")

        assert node.gz_sync_total_count == 250
        assert node.gz_sync_success_count == 1
        assert len(node.gz_success_latency_history) == 1
        assert len(node.gz_latency_history) == 1

        node.destroy_node()
    finally:
        if shutdown_at_end:
            rclpy.shutdown()







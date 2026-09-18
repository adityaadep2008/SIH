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




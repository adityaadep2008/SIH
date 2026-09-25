"""Unit tests for Gazebo pose verifier and carrier forensic observability."""

import math
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock
import pytest

scripts_dir = Path(__file__).resolve().parent.parent.parent.parent / "scripts"
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

from verify_gazebo_pose import check_pose_tolerance, verify_pose, wrap_angle


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_check_pose_tolerance_success():
    pose = {"x": 2.01, "y": 3.02, "z": 0.03, "yaw": 0.05, "tilt": 0.01}
    is_ok, err_dict, failures = check_pose_tolerance(
        pose, expected_x=2.0, expected_y=3.0, expected_yaw=0.0,
        tol_xy=0.25, tol_z=0.15, tol_yaw=0.35, tol_tilt=0.20
    )
    assert is_ok is True
    assert len(failures) == 0
    assert err_dict["err_xy"] < 0.25
    assert err_dict["err_yaw"] < 0.35


def test_check_pose_tolerance_position_mismatch():
    pose = {"x": 2.50, "y": 3.00, "z": 0.03, "yaw": 0.0, "tilt": 0.0}
    is_ok, err_dict, failures = check_pose_tolerance(
        pose, expected_x=2.0, expected_y=3.0, expected_yaw=0.0,
        tol_xy=0.25
    )
    assert is_ok is False
    assert any("POSITION_MISMATCH" in f for f in failures)


def test_check_pose_tolerance_yaw_mismatch():
    pose = {"x": 2.0, "y": 3.0, "z": 0.03, "yaw": 1.57, "tilt": 0.0}
    is_ok, err_dict, failures = check_pose_tolerance(
        pose, expected_x=2.0, expected_y=3.0, expected_yaw=0.0,
        tol_yaw=0.35
    )
    assert is_ok is False
    assert any("YAW_MISMATCH" in f for f in failures)


def test_check_pose_tolerance_elevation_mismatch():
    pose = {"x": 2.0, "y": 3.0, "z": 0.35, "yaw": 0.0, "tilt": 0.0}
    is_ok, err_dict, failures = check_pose_tolerance(
        pose, expected_x=2.0, expected_y=3.0, expected_yaw=0.0, expected_z=0.03,
        tol_z=0.15
    )
    assert is_ok is False
    assert any("ELEVATION_MISMATCH" in f for f in failures)


def test_check_pose_tolerance_tilt_exceeded():
    pose = {"x": 2.0, "y": 3.0, "z": 0.03, "yaw": 0.0, "tilt": 0.45}
    is_ok, err_dict, failures = check_pose_tolerance(
        pose, expected_x=2.0, expected_y=3.0, expected_yaw=0.0,
        tol_tilt=0.20
    )
    assert is_ok is False
    assert any("TILT_EXCEEDED" in f for f in failures)


def test_verify_pose_cli_recovery():
    """Verify that when transport is absent, CLI query is scheduled after grace period and succeeds."""
    mock_cli = MagicMock(return_value={
        "x": 1.0, "y": 2.0, "z": 0.03, "roll": 0.0, "pitch": 0.0,
        "yaw": 0.0, "tilt": 0.0, "source": "mock_cli"
    })

    # Run with short timeout and simulated clock/sleep
    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=3.5,
        query_cli_fn=mock_cli
    )
    assert ret == 0
    assert mock_cli.called


def test_verify_pose_timeout_no_pose():
    """Verify that when neither transport nor CLI returns a pose, timeout returns 1."""
    mock_cli = MagicMock(return_value=None)
    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=2.2,
        query_cli_fn=mock_cli
    )
    assert ret == 1


def test_verify_pose_cli_rate_limiting():
    """Verify CLI is called at most once per second during fallback."""
    call_times = []

    def mock_cli_recording(robot):
        call_times.append(time.time())
        return None

    verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=4.2,
        query_cli_fn=mock_cli_recording
    )

    assert len(call_times) >= 2
    # Verify interval between consecutive calls is >= 0.9s
    for i in range(1, len(call_times)):
        interval = call_times[i] - call_times[i - 1]
        assert interval >= 0.9, f"CLI called too rapidly: interval={interval:.3f}s"


def test_verifier_unsubscribes_on_transport_success():
    """Verify that both Gazebo topics are unsubscribed upon transport verification success."""
    mock_gz_node = MagicMock()
    callback_holder = []

    def mock_sub(msg_type, topic, cb):
        callback_holder.append((topic, cb))

    mock_gz_node.subscribe = mock_sub

    class MockPose:
        name = "robot_1/turtlebot4"
        class position:
            x, y, z = 1.0, 2.0, 0.03
        class orientation:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0

    class MockMsg:
        pose = [MockPose()]

    def custom_sleep(dt):
        # Trigger callback on first sleep
        if callback_holder:
            for topic, cb in callback_holder:
                cb(MockMsg())

    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=2.0,
        sleep_fn=custom_sleep,
        gz_node=mock_gz_node
    )

    assert ret == 0
    # Both topics must be explicitly unsubscribed
    unsub_topics = [call.args[0] for call in mock_gz_node.unsubscribe.call_args_list]
    assert "/world/default/dynamic_pose/info" in unsub_topics
    assert "/world/default/pose/info" in unsub_topics


def test_verifier_unsubscribes_on_mismatch():
    """Verify that both topics are unsubscribed when physical coordinate mismatch is detected."""
    mock_gz_node = MagicMock()
    callback_holder = []
    mock_gz_node.subscribe = lambda msg_type, topic, cb: callback_holder.append(cb)

    class MockPose:
        name = "robot_1/turtlebot4"
        class position:
            x, y, z = 5.0, 5.0, 0.03  # Far from expected (1.0, 2.0)
        class orientation:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0

    class MockMsg:
        pose = [MockPose()]

    def custom_sleep(dt):
        for cb in callback_holder:
            cb(MockMsg())

    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=0.1,
        sleep_fn=custom_sleep,
        gz_node=mock_gz_node
    )

    assert ret == 1
    unsub_topics = [call.args[0] for call in mock_gz_node.unsubscribe.call_args_list]
    assert "/world/default/dynamic_pose/info" in unsub_topics
    assert "/world/default/pose/info" in unsub_topics


def test_verifier_unsubscribes_on_timeout():
    """Verify that both topics are unsubscribed when verifier times out."""
    mock_gz_node = MagicMock()
    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=0.05,
        gz_node=mock_gz_node
    )
    assert ret == 1
    unsub_topics = [call.args[0] for call in mock_gz_node.unsubscribe.call_args_list]
    assert "/world/default/dynamic_pose/info" in unsub_topics
    assert "/world/default/pose/info" in unsub_topics


def test_verifier_unsubscribes_on_exception():
    """Verify that both topics are unsubscribed even if an unexpected exception occurs inside check."""
    mock_gz_node = MagicMock()
    with pytest.raises(RuntimeError):
        verify_pose(
            robot_name="robot_1",
            expected_x=1.0,
            expected_y=2.0,
            expected_yaw=0.0,
            timeout=1.0,
            sleep_fn=MagicMock(side_effect=RuntimeError("Simulated unexpected failure")),
            gz_node=mock_gz_node
        )

    unsub_topics = [call.args[0] for call in mock_gz_node.unsubscribe.call_args_list]
    assert "/world/default/dynamic_pose/info" in unsub_topics
    assert "/world/default/pose/info" in unsub_topics


def test_verifier_rejects_stale_samples():
    """Verify that verifier rejects pose samples timestamped before verifier launch."""
    mock_gz_node = MagicMock()
    callback_holder = []
    mock_gz_node.subscribe = lambda msg_type, topic, cb: callback_holder.append(cb)

    # Simulate CLI returning sample with timestamp in the past
    mock_cli = MagicMock(return_value={
        "x": 1.0, "y": 2.0, "z": 0.03, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "tilt": 0.0,
        "source": "stale_cli", "timestamp": time.time() - 100.0  # Old sample!
    })

    ret = verify_pose(
        robot_name="robot_1",
        expected_x=1.0,
        expected_y=2.0,
        expected_yaw=0.0,
        timeout=2.2,
        query_cli_fn=mock_cli,
        gz_node=mock_gz_node
    )

    # Must not accept the stale sample as a valid result
    assert ret == 1


def test_verifier_concurrent_callback_teardown_race():
    """Verify thread-safety when concurrent callbacks fire continuously during teardown."""
    import threading
    mock_gz_node = MagicMock()
    callbacks = []
    mock_gz_node.subscribe = lambda msg_type, topic, cb: callbacks.append(cb)

    class MockPose:
        name = "robot_1/turtlebot4"
        class position:
            x, y, z = 1.0, 2.0, 0.03
        class orientation:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0

    class MockMsg:
        pose = [MockPose()]

    stop_threads = threading.Event()

    def worker_thread(cb):
        while not stop_threads.is_set():
            try:
                cb(MockMsg())
            except Exception:
                pass
            time.sleep(0.001)

    threads = []

    def custom_sleep(dt):
        nonlocal threads
        if not threads and callbacks:
            for _ in range(4):
                t = threading.Thread(target=worker_thread, args=(callbacks[0],))
                t.daemon = True
                t.start()
                threads.append(t)
        time.sleep(0.02)

    try:
        ret = verify_pose(
            robot_name="robot_1",
            expected_x=1.0,
            expected_y=2.0,
            expected_yaw=0.0,
            timeout=2.0,
            sleep_fn=custom_sleep,
            gz_node=mock_gz_node
        )
        assert ret == 0
    finally:
        stop_threads.set()
        for t in threads:
            t.join(timeout=1.0)

    unsub_topics = [call.args[0] for call in mock_gz_node.unsubscribe.call_args_list]
    assert "/world/default/dynamic_pose/info" in unsub_topics
    assert "/world/default/pose/info" in unsub_topics


def test_verifier_drain_timeout_failure():
    """Verify that if in-flight callbacks fail to drain within 0.5s, verifier exits with code 1."""
    from unittest.mock import patch
    mock_gz_node = MagicMock()
    callbacks = []
    mock_gz_node.subscribe = lambda msg_type, topic, cb: callbacks.append(cb)

    class MockPose:
        name = "robot_1/turtlebot4"
        class position:
            x, y, z = 1.0, 2.0, 0.03
        class orientation:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0

    class MockMsg:
        pose = [MockPose()]

    def custom_sleep(dt):
        if callbacks:
            callbacks[0](MockMsg())
        time.sleep(0.01)

    with patch("threading.Condition.wait_for", return_value=False):
        ret = verify_pose(
            robot_name="robot_1",
            expected_x=1.0,
            expected_y=2.0,
            expected_yaw=0.0,
            timeout=1.0,
            sleep_fn=custom_sleep,
            gz_node=mock_gz_node
        )
        assert ret == 1



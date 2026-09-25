#!/usr/bin/env python3
"""Verify that a spawned AMR matches expected Gazebo ground-truth coordinates and orientation.

Queries Gazebo Sim transport and gz model CLI directly to inspect the model's physical world pose.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

# Ensure Gazebo Sim transport binds to localhost if GZ_IP is not set in environment
os.environ.setdefault("GZ_IP", "127.0.0.1")


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def query_gz_model_cli(robot_name):
    """Query model pose via 'gz model -m <name> -p' CLI service call."""
    for model_name in [f"{robot_name}/turtlebot4", robot_name]:
        try:
            res = subprocess.run(
                ["gz", "model", "-m", model_name, "-p"],
                capture_output=True, text=True, timeout=6.0
            )
            if res.returncode == 0 and res.stdout:
                m = re.search(
                    r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]",
                    res.stdout
                )
                if m:
                    x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                    roll, pitch, yaw = float(m.group(4)), float(m.group(5)), float(m.group(6))
                    return {
                        "x": x, "y": y, "z": z,
                        "roll": roll, "pitch": pitch, "yaw": yaw,
                        "tilt": math.hypot(roll, pitch),
                        "source": f"gz_model_cli({model_name})",
                        "timestamp": time.time()
                    }
        except Exception:
            pass
    return None


import concurrent.futures


def check_pose_tolerance(pose, expected_x, expected_y, expected_yaw, expected_z=0.03,
                         tol_xy=0.25, tol_z=0.15, tol_yaw=0.35, tol_tilt=0.20):
    """Evaluate pose against tolerances and return (is_ok, err_dict, failure_reasons)."""
    cur_x = pose["x"]
    cur_y = pose["y"]
    cur_z = pose["z"]
    cur_yaw = pose["yaw"]
    cur_tilt = pose["tilt"]

    err_x = abs(cur_x - expected_x)
    err_y = abs(cur_y - expected_y)
    err_xy = math.hypot(err_x, err_y)
    err_z = abs(cur_z - expected_z)
    err_yaw = abs(wrap_angle(cur_yaw - expected_yaw))

    failures = []
    if err_xy > tol_xy:
        failures.append(f"POSITION_MISMATCH(err_xy={err_xy:.3f}m > tol={tol_xy:.3f}m)")
    if err_z > tol_z:
        failures.append(f"ELEVATION_MISMATCH(err_z={err_z:.3f}m > tol={tol_z:.3f}m)")
    if err_yaw > tol_yaw:
        failures.append(f"YAW_MISMATCH(err_yaw={err_yaw:.3f}rad > tol={tol_yaw:.3f}rad)")
    if cur_tilt > tol_tilt:
        failures.append(f"TILT_EXCEEDED(tilt={cur_tilt:.3f}rad > tol={tol_tilt:.3f}rad)")

    err_dict = {
        "err_xy": err_xy,
        "err_z": err_z,
        "err_yaw": err_yaw,
        "tilt": cur_tilt
    }
    return (len(failures) == 0, err_dict, failures)


def verify_pose(robot_name, expected_x, expected_y, expected_yaw, expected_z=0.03,
                timeout=15.0, tol_xy=0.25, tol_z=0.15, tol_yaw=0.35, tol_tilt=0.20,
                query_cli_fn=None, sleep_fn=time.sleep, gz_node=None):
    """Asynchronously verify Gazebo ground-truth pose with safe lifecycle teardown."""
    if query_cli_fn is None:
        query_cli_fn = query_gz_model_cli

    latest_pose = None
    transport_samples = 0
    cli_samples = 0

    topics = [
        "/world/default/dynamic_pose/info",
        "/world/default/pose/info"
    ]

    import threading
    is_active = True
    callback_cond = threading.Condition()
    in_flight_callbacks = 0

    created_gz_node = False
    if gz_node is None:
        try:
            from gz.transport13 import Node as GzNode
            gz_node = GzNode()
            created_gz_node = True
        except Exception:
            gz_node = None

    start_t = time.time()

    def on_pose_msg(msg):
        nonlocal latest_pose, transport_samples, in_flight_callbacks
        with callback_cond:
            if not is_active:
                return
            in_flight_callbacks += 1

        try:
            sample_t = time.time()
            if sample_t < start_t:
                return

            for p in msg.pose:
                if p.name in (f"{robot_name}/turtlebot4", robot_name):
                    roll, pitch, yaw = quaternion_to_euler(p.orientation)
                    latest_pose = {
                        "x": float(p.position.x),
                        "y": float(p.position.y),
                        "z": float(p.position.z),
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "tilt": math.hypot(roll, pitch),
                        "source": "gz_transport",
                        "timestamp": sample_t
                    }
                    transport_samples += 1
        finally:
            with callback_cond:
                in_flight_callbacks -= 1
                callback_cond.notify_all()

    if gz_node is not None:
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            for top in topics:
                gz_node.subscribe(Pose_V, top, on_pose_msg)
        except Exception:
            pass

    cli_executor = None
    cli_future = None
    last_cli_call_t = 0.0

    exit_code = 1
    pending_success_msg = None
    pending_error_msgs = []

    try:
        while time.time() - start_t < timeout:
            # Check completed CLI query if pending
            if cli_future is not None and cli_future.done():
                try:
                    cli_result = cli_future.result()
                    if cli_result:
                        cli_result.setdefault("timestamp", time.time())
                        if cli_result.get("timestamp", 0) >= start_t:
                            cli_samples += 1
                            latest_pose = cli_result
                except Exception:
                    pass
                cli_future = None

            # Schedule asynchronous CLI query lazily if grace window (2.0s) has passed
            now = time.time()
            if (now - start_t) >= 2.0 and cli_future is None and (now - last_cli_call_t) >= 1.0:
                last_cli_call_t = now
                if cli_executor is None:
                    cli_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
                cli_future = cli_executor.submit(query_cli_fn, robot_name)

            # Evaluate latest pose against tolerances if any sample exists and is fresh
            if latest_pose is not None and latest_pose.get("timestamp", 0) >= start_t:
                is_ok, err_dict, _ = check_pose_tolerance(
                    latest_pose, expected_x, expected_y, expected_yaw, expected_z,
                    tol_xy, tol_z, tol_yaw, tol_tilt
                )
                if is_ok:
                    elapsed = time.time() - start_t
                    pending_success_msg = (
                        f"[GAZEBO POSE OK] {robot_name}: actual=(x={latest_pose['x']:.3f}, y={latest_pose['y']:.3f}, "
                        f"z={latest_pose['z']:.3f}, yaw={latest_pose['yaw']:.3f}rad, tilt={latest_pose['tilt']:.3f}rad) "
                        f"matches expected=(x={expected_x:.3f}, y={expected_y:.3f}, yaw={expected_yaw:.3f}rad) "
                        f"[err_xy={err_dict['err_xy']:.3f}m, err_yaw={err_dict['err_yaw']:.3f}rad, "
                        f"src={latest_pose.get('source', 'unknown')}, transport_samples={transport_samples}, "
                        f"cli_samples={cli_samples}, latency={elapsed:.2f}s]"
                    )
                    exit_code = 0
                    break

            sleep_fn(0.05)

        if exit_code != 0:
            # Timeout reached: evaluate final verdict
            # Harvest any in-flight CLI result before concluding
            if cli_future is not None:
                try:
                    final_cli = cli_future.result(timeout=1.0)
                    if final_cli:
                        final_cli.setdefault("timestamp", time.time())
                        if final_cli.get("timestamp", 0) >= start_t:
                            cli_samples += 1
                            latest_pose = final_cli
                except Exception:
                    pass

            if latest_pose is not None and latest_pose.get("timestamp", 0) >= start_t:
                is_ok, err_dict, failures = check_pose_tolerance(
                    latest_pose, expected_x, expected_y, expected_yaw, expected_z,
                    tol_xy, tol_z, tol_yaw, tol_tilt
                )
                if is_ok:
                    pending_success_msg = (
                        f"[GAZEBO POSE OK] {robot_name}: actual=(x={latest_pose['x']:.3f}, y={latest_pose['y']:.3f}, "
                        f"z={latest_pose['z']:.3f}, yaw={latest_pose['yaw']:.3f}rad, tilt={latest_pose['tilt']:.3f}rad) "
                        f"[src={latest_pose.get('source')}, transport_samples={transport_samples}, cli_samples={cli_samples}]"
                    )
                    exit_code = 0
                else:
                    failure_str = ", ".join(failures)
                    msg_err = (
                        f"[GAZEBO POSE MISMATCH] {robot_name}: expected=(x={expected_x:.3f}, y={expected_y:.3f}, yaw={expected_yaw:.3f}rad), "
                        f"actual=(x={latest_pose['x']:.3f}, y={latest_pose['y']:.3f}, z={latest_pose['z']:.3f}, "
                        f"yaw={latest_pose['yaw']:.3f}rad, tilt={latest_pose['tilt']:.3f}rad) "
                        f"[err_xy={err_dict['err_xy']:.3f}m, err_yaw={err_dict['err_yaw']:.3f}rad, err_z={err_dict['err_z']:.3f}m, "
                        f"failures=[{failure_str}], src={latest_pose.get('source')}, "
                        f"transport_samples={transport_samples}, cli_samples={cli_samples}]"
                    )
                    pending_error_msgs.append(msg_err)
                    exit_code = 1
            else:
                msg_timeout = (
                    f"[GAZEBO POSE TIMEOUT] {robot_name}: no pose data received from Gazebo transport or CLI within {timeout}s. "
                    f"[GAZEBO POSE ERROR: NO_POSE_OBSERVED, transport_samples=0, cli_samples={cli_samples}]"
                )
                pending_error_msgs.append(msg_timeout)
                exit_code = 1

    finally:
        # 1. Atomically deactivate callbacks under lock
        with callback_cond:
            is_active = False

        # 2. Unsubscribe from Gazebo transport topics outside the lock
        if gz_node is not None:
            for top in topics:
                try:
                    gz_node.unsubscribe(top)
                except Exception:
                    pass

        # 3. Wait with bounded deadline for any in-flight C++ callback to finish
        with callback_cond:
            drained = callback_cond.wait_for(lambda: in_flight_callbacks == 0, timeout=0.5)
            if not drained:
                teardown_err = (
                    f"[GAZEBO POSE ERROR: TEARDOWN_DRAIN_TIMEOUT] {robot_name}: "
                    f"{in_flight_callbacks} callback(s) still in-flight after 0.5s."
                )
                pending_error_msgs.append(teardown_err)
                pending_success_msg = None
                exit_code = 1

        # 4. Explicitly release node reference if created locally
        if created_gz_node:
            gz_node = None

        # 5. Shut down lazy CLI executor cleanly if created
        if cli_executor is not None:
            cli_executor.shutdown(wait=True, cancel_futures=True)

        # 6. Output messages only after complete and clean teardown
        if pending_success_msg is not None:
            print(pending_success_msg)
            sys.stdout.flush()
        for msg in pending_error_msgs:
            print(msg)
            print(msg, file=sys.stderr)
        sys.stdout.flush()
        sys.stderr.flush()

    return exit_code


def main():
    parser = argparse.ArgumentParser(description="Verify AMR Gazebo ground truth pose")
    parser.add_argument("--robot", type=str, required=True, help="Robot ID (e.g. robot_1)")
    parser.add_argument("--expected-x", type=float, required=True, help="Expected world X coordinate")
    parser.add_argument("--expected-y", type=float, required=True, help="Expected world Y coordinate")
    parser.add_argument("--expected-yaw", type=float, required=True, help="Expected yaw in radians")
    parser.add_argument("--expected-z", type=float, default=0.03, help="Expected Z coordinate (default: 0.03)")
    parser.add_argument("--timeout", type=float, default=15.0, help="Wait timeout in seconds (default: 15.0)")
    parser.add_argument("--tol-xy", type=float, default=0.25, help="XY position tolerance in meters (default: 0.25)")
    parser.add_argument("--tol-z", type=float, default=0.15, help="Z position tolerance in meters (default: 0.15)")
    parser.add_argument("--tol-yaw", type=float, default=0.35, help="Yaw tolerance in radians (default: 0.35)")
    parser.add_argument("--tol-tilt", type=float, default=0.20, help="Max tilt (pitch/roll) in radians (default: 0.20)")
    args = parser.parse_args()

    return verify_pose(
        robot_name=args.robot,
        expected_x=args.expected_x,
        expected_y=args.expected_y,
        expected_yaw=args.expected_yaw,
        expected_z=args.expected_z,
        timeout=args.timeout,
        tol_xy=args.tol_xy,
        tol_z=args.tol_z,
        tol_yaw=args.tol_yaw,
        tol_tilt=args.tol_tilt
    )


if __name__ == "__main__":
    sys.exit(main())


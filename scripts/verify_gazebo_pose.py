#!/usr/bin/env python3
"""Verify that a spawned AMR matches expected Gazebo ground-truth coordinates and orientation.

Queries Gazebo Sim transport and gz model CLI directly to inspect the model's physical world pose.
"""

import argparse
import math
import re
import subprocess
import sys
import time


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
                capture_output=True, text=True, timeout=1.5
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
                        "source": f"gz_model_cli({model_name})"
                    }
        except Exception:
            pass
    return None


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

    latest_pose = {}
    found_event = False

    # Attempt Gazebo transport subscription in background
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        gz_node = GzNode()

        def on_pose_msg(msg):
            nonlocal latest_pose, found_event
            for p in msg.pose:
                name = p.name
                if p.name in (f"{args.robot}/turtlebot4", args.robot):
                    roll, pitch, yaw = quaternion_to_euler(p.orientation)
                    latest_pose = {
                        "x": float(p.position.x),
                        "y": float(p.position.y),
                        "z": float(p.position.z),
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "tilt": math.hypot(roll, pitch),
                        "source": "gz_transport"
                    }
                    found_event = True

        gz_node.subscribe(Pose_V, "/world/default/dynamic_pose/info", on_pose_msg)
        gz_node.subscribe(Pose_V, "/world/default/pose/info", on_pose_msg)
    except Exception as exc:
        # Fallback to CLI queries
        pass

    start_t = time.time()
    while time.time() - start_t < args.timeout:
        # Try transport sample first
        pose_sample = latest_pose if (found_event and latest_pose) else None

        # Give gz_transport a brief grace period (0.6s) to receive streaming packets before blocking on CLI queries
        if pose_sample is None and (time.time() - start_t) > 0.6:
            cli_sample = query_gz_model_cli(args.robot)
            if cli_sample:
                pose_sample = cli_sample
                latest_pose = cli_sample

        if pose_sample:
            cur_x = pose_sample["x"]
            cur_y = pose_sample["y"]
            cur_z = pose_sample["z"]
            cur_yaw = pose_sample["yaw"]
            cur_tilt = pose_sample["tilt"]

            err_x = abs(cur_x - args.expected_x)
            err_y = abs(cur_y - args.expected_y)
            err_xy = math.hypot(err_x, err_y)
            err_z = abs(cur_z - args.expected_z)
            err_yaw = abs(wrap_angle(cur_yaw - args.expected_yaw))

            if err_xy <= args.tol_xy and err_z <= args.tol_z and err_yaw <= args.tol_yaw and cur_tilt <= args.tol_tilt:
                msg_ok = (
                    f"[GAZEBO POSE OK] {args.robot}: actual=(x={cur_x:.3f}, y={cur_y:.3f}, z={cur_z:.3f}, "
                    f"yaw={cur_yaw:.3f}rad, tilt={cur_tilt:.3f}rad) "
                    f"matches expected=(x={args.expected_x:.3f}, y={args.expected_y:.3f}, yaw={args.expected_yaw:.3f}rad) "
                    f"[err_xy={err_xy:.3f}m, err_yaw={err_yaw:.3f}rad, src={pose_sample.get('source', 'gz')}]"
                )
                print(msg_ok)
                sys.stdout.flush()
                return 0

        time.sleep(0.05)

    if latest_pose:
        cur_x = latest_pose["x"]
        cur_y = latest_pose["y"]
        cur_z = latest_pose["z"]
        cur_yaw = latest_pose["yaw"]
        cur_tilt = latest_pose["tilt"]
        err_xy = math.hypot(cur_x - args.expected_x, cur_y - args.expected_y)
        err_yaw = abs(wrap_angle(cur_yaw - args.expected_yaw))
        msg_err = (
            f"[GAZEBO POSE MISMATCH] {args.robot}: expected=(x={args.expected_x:.3f}, y={args.expected_y:.3f}, yaw={args.expected_yaw:.3f}), "
            f"actual=(x={cur_x:.3f}, y={cur_y:.3f}, z={cur_z:.3f}, yaw={cur_yaw:.3f}rad, tilt={cur_tilt:.3f}rad) "
            f"[err_xy={err_xy:.3f}m, err_yaw={err_yaw:.3f}rad, err_z={abs(cur_z - args.expected_z):.3f}m]"
        )
        print(msg_err)
        print(msg_err, file=sys.stderr)
    else:
        msg_timeout = f"[GAZEBO POSE TIMEOUT] {args.robot}: no pose data received from Gazebo within {args.timeout}s."
        print(msg_timeout)
        print(msg_timeout, file=sys.stderr)

    sys.stdout.flush()
    sys.stderr.flush()
    return 1


if __name__ == "__main__":
    sys.exit(main())

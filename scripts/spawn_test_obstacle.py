#!/usr/bin/env python3
"""Spawn a targeted test obstacle in Gazebo directly intersecting an AMR's planned path.

Forces the targeted AMR to detect the blockage and recalculate its route in real-time.
"""

import argparse
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Auto-add local workspace packages to sys.path
WORKSPACE_INSTALL = Path("/home/rtsws/amr_ws/install")
if WORKSPACE_INSTALL.exists():
    for sp in WORKSPACE_INSTALL.glob("**/site-packages"):
        if str(sp) not in sys.path:
            sys.path.insert(0, str(sp))


def detect_active_ros_environment() -> Dict[str, str]:
    """Scan active simulation processes to discover ROS_DOMAIN_ID and CycloneDDS settings."""
    env_vars = {}
    proc_dir = Path("/proc")
    if proc_dir.exists():
        for p in proc_dir.glob("[0-9]*"):
            try:
                cmdline_path = p / "cmdline"
                if not cmdline_path.exists():
                    continue
                cmdline = cmdline_path.read_text(errors="ignore")
                if any(k in cmdline for k in ["robot_agent", "data_collection_node", "warehouse_map_node", "gz sim"]):
                    environ_path = p / "environ"
                    if environ_path.exists():
                        env_data = environ_path.read_bytes().split(b"\x00")
                        for item in env_data:
                            try:
                                key, val = item.decode("utf-8").split("=", 1)
                                if key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_AUTOMATIC_DISCOVERY_RANGE"):
                                    env_vars[key] = val
                            except ValueError:
                                pass
                        if "ROS_DOMAIN_ID" in env_vars:
                            break
            except (PermissionError, ProcessLookupError, FileNotFoundError):
                continue
    return env_vars


# Apply active simulation environment
_detected_env = detect_active_ros_environment()
for _k, _v in _detected_env.items():
    if _k not in os.environ:
        os.environ[_k] = _v

MODEL_PRESETS = {
    "pallet_boxes": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringA_01/model.sdf",
    "clutter_c": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringC_01/model.sdf",
    "clutter_d": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringD_01/model.sdf",
    "bucket": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_Bucket_01/model.sdf",
    "trash_can": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_TrashCanC_01/model.sdf",
}

STRATEGIC_CHOKEPOINTS = [
    {"name": "Central Main Corridor Intersection", "x": 0.0, "y": 0.0},
    {"name": "North-Central Transit Corridor", "x": 0.0, "y": 15.0},
    {"name": "South-Central Transit Corridor", "x": 0.0, "y": -15.0},
    {"name": "West Storage Aisle Entrance", "x": -7.5, "y": 0.0},
    {"name": "East Storage Aisle Entrance", "x": 7.5, "y": 0.0},
]


def normalize_robot_id(name: str) -> str:
    """Normalize user input like '1', 'amr1', 'robot_1' into standard 'robot_1'."""
    name_clean = name.strip().lower()
    if name_clean in ("auto", "any", "all"):
        return "auto"
    if name_clean.isdigit():
        return f"robot_{name_clean}"
    if name_clean.startswith("amr"):
        num = name_clean.replace("amr", "").strip("_")
        if num.isdigit():
            return f"robot_{num}"
    if name_clean.startswith("robot_"):
        return name_clean
    return name_clean


def compute_lookahead_intersection(
    waypoints: List[Tuple[float, float]],
    current_pose: Optional[Tuple[float, float, float]],
    lookahead_dist: float
) -> Tuple[float, float, str]:
    """Compute targeted (x, y) intersection coordinate along the route at lookahead_dist ahead."""
    if not waypoints:
        if current_pose is not None:
            rx, ry, yaw = current_pose
            tx = rx + lookahead_dist * math.cos(yaw)
            ty = ry + lookahead_dist * math.sin(yaw)
            return round(tx, 2), round(ty, 2), "Heading Projection (No Waypoints)"
        cp = random.choice(STRATEGIC_CHOKEPOINTS)
        return cp["x"], cp["y"], f"Strategic Chokepoint ({cp['name']})"

    if len(waypoints) == 1:
        return round(waypoints[0][0], 2), round(waypoints[0][1], 2), "Single Target Waypoint"

    if current_pose is None:
        start_pt = waypoints[0]
        curr_seg_idx = 0
    else:
        rx, ry, _ = current_pose
        # Project robot pose onto the closest path segment
        best_dist_sq = float("inf")
        best_proj = waypoints[0]
        best_seg_idx = 0

        for i in range(len(waypoints) - 1):
            p0 = waypoints[i]
            p1 = waypoints[i + 1]
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                proj = p0
            else:
                t = max(0.0, min(1.0, ((rx - p0[0]) * dx + (ry - p0[1]) * dy) / seg_len_sq))
                proj = (p0[0] + t * dx, p0[1] + t * dy)
            d_sq = (rx - proj[0]) ** 2 + (ry - proj[1]) ** 2
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_proj = proj
                best_seg_idx = i

        start_pt = best_proj
        curr_seg_idx = best_seg_idx

    # Step forward from start_pt along the path by lookahead_dist
    p0 = start_pt
    p1 = waypoints[curr_seg_idx + 1]
    first_seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    if lookahead_dist <= first_seg_len:
        ratio = lookahead_dist / first_seg_len if first_seg_len > 1e-4 else 0.0
        ix = p0[0] + ratio * (p1[0] - p0[0])
        iy = p0[1] + ratio * (p1[1] - p0[1])
        return round(ix, 2), round(iy, 2), f"Planned Route Trajectory (+{lookahead_dist:.1f}m ahead)"

    accum = first_seg_len
    for i in range(curr_seg_idx + 1, len(waypoints) - 1):
        sp0 = waypoints[i]
        sp1 = waypoints[i + 1]
        slen = math.hypot(sp1[0] - sp0[0], sp1[1] - sp0[1])
        if accum + slen >= lookahead_dist:
            remain = lookahead_dist - accum
            ratio = remain / slen if slen > 1e-4 else 0.0
            ix = sp0[0] + ratio * (sp1[0] - sp0[0])
            iy = sp0[1] + ratio * (sp1[1] - sp0[1])
            return round(ix, 2), round(iy, 2), f"Planned Route Trajectory (+{lookahead_dist:.1f}m ahead)"
        accum += slen

    # If lookahead exceeds remaining path, return terminal waypoint
    last_x, last_y = waypoints[-1]
    return round(last_x, 2), round(last_y, 2), f"Planned Route Terminal Waypoint (+{accum:.1f}m ahead)"


class TrajectoryListener:
    """Listens to active ROS 2 RoutePlan and RobotState topics to find AMR trajectory."""

    def __init__(self, target_robot: str, timeout_s: float):
        self.target_robot = target_robot
        self.timeout_s = timeout_s
        self.active_routes: Dict[str, List[Tuple[float, float]]] = {}
        self.active_poses: Dict[str, Tuple[float, float, float]] = {}
        self.route_received = False

    def discover_target(self) -> Tuple[str, Optional[List[Tuple[float, float]]], Optional[Tuple[float, float, float]]]:
        """Spin briefly to capture live route and pose data."""
        try:
            import rclpy
            from rclpy.node import Node
            from sih_amr_interfaces.msg import RoutePlan, RobotState
        except ImportError:
            print("[WARN] ROS 2 or sih_amr_interfaces not importable in current environment.")
            return self.target_robot, None, None

        if not rclpy.ok():
            rclpy.init()

        node = Node("targeted_obstacle_spawner_probe")
        candidate_robots = ["robot_1", "robot_2", "robot_3", "robot_4"] if self.target_robot == "auto" else [self.target_robot]

        def make_route_cb(r_id):
            def cb(msg: RoutePlan):
                if msg.waypoints:
                    pts = [(wp.x, wp.y) for wp in msg.waypoints]
                    self.active_routes[r_id] = pts
                    self.route_received = True
            return cb

        def on_fleet_state(msg: RobotState):
            r_id = msg.fleet_header.robot_id
            if r_id and msg.localization_valid:
                self.active_poses[r_id] = (msg.pose.x, msg.pose.y, msg.pose.theta)

        # Create subscribers
        subs = []
        for r_id in candidate_robots:
            subs.append(node.create_subscription(RoutePlan, f"/{r_id}/planned_route", make_route_cb(r_id), 10))
        subs.append(node.create_subscription(RobotState, "/fleet/robot_state", on_fleet_state, 10))

        start_time = time.time()
        while time.time() - start_time < self.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Stop early if we have both route and pose for our desired robot
            if self.target_robot != "auto":
                if self.target_robot in self.active_routes and self.target_robot in self.active_poses:
                    break
            elif self.active_routes:
                break

        node.destroy_node()

        # Resolve selected robot
        selected_robot = self.target_robot
        if self.target_robot == "auto":
            # Pick first robot with active route, or first robot with pose
            if self.active_routes:
                selected_robot = list(self.active_routes.keys())[0]
            elif self.active_poses:
                selected_robot = list(self.active_poses.keys())[0]
            else:
                selected_robot = "robot_1"

        return (
            selected_robot,
            self.active_routes.get(selected_robot),
            self.active_poses.get(selected_robot)
        )


def main():
    parser = argparse.ArgumentParser(
        description="Spawn a targeted obstacle in Gazebo directly intersecting an AMR's planned path."
    )
    parser.add_argument("-r", "--robot", type=str, default="robot_1",
                        help="Target robot to intercept: robot_1..robot_4, amr1..amr4, 1..4, or 'auto' (default: robot_1)")
    parser.add_argument("-d", "--ahead", type=float, default=3.0,
                        help="Lookahead distance in meters ahead of the AMR along its route (default: 3.0 m)")
    parser.add_argument("-x", type=float, default=None, help="Explicit X coordinate override in map frame")
    parser.add_argument("-y", type=float, default=None, help="Explicit Y coordinate override in map frame")
    parser.add_argument("-z", type=float, default=0.05, help="Z coordinate in map frame (default: 0.05)")
    parser.add_argument("--model", choices=list(MODEL_PRESETS.keys()), default="pallet_boxes",
                        help="Obstacle model type to spawn (default: pallet_boxes)")
    parser.add_argument("--name", type=str, default=None, help="Unique entity name in Gazebo")
    parser.add_argument("-t", "--timeout", type=float, default=5.0,
                        help="Timeout in seconds to capture live AMR trajectory (default: 5.0 s)")
    args = parser.parse_args()

    norm_robot = normalize_robot_id(args.robot)
    target_info = "User-Specified Coordinate"

    if args.x is not None and args.y is not None:
        x, y = args.x, args.y
        resolved_robot = norm_robot
    else:
        print(f"[*] Discovering trajectory for target robot '{norm_robot}' (timeout: {args.timeout:.1f}s)...")
        listener = TrajectoryListener(norm_robot, args.timeout)
        resolved_robot, waypoints, pose = listener.discover_target()

        if pose:
            print(f"[*] Found live pose for {resolved_robot}: x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={pose[2]:.2f} rad")
        if waypoints:
            print(f"[*] Found active route for {resolved_robot} with {len(waypoints)} waypoints.")

        x, y, target_info = compute_lookahead_intersection(waypoints, pose, args.ahead)

    obs_name = args.name or f"target_obs_{resolved_robot}_{int(time.time())}"
    model_file = MODEL_PRESETS[args.model]

    print(f"\n========================================================")
    print(f" Spawning Targeted Obstacle into Gazebo:")
    print(f" • Name:            {obs_name}")
    print(f" • Model:           {args.model} ({Path(model_file).name})")
    print(f" • Target AMR:      {resolved_robot}")
    print(f" • Intercept Spot:  x = {x:.2f} m, y = {y:.2f} m, z = {args.z:.2f} m")
    print(f" • Strategy:        {target_info}")
    print(f"========================================================\n")

    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", "default",
        "-file", model_file,
        "-name", obs_name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(args.z),
        "-allow_renaming", "true"
    ]

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(f"✔ Successfully spawned targeted obstacle '{obs_name}' at ({x:.2f}, {y:.2f})!")
        print(f"  This obstacle now intersects {resolved_robot}'s path, forcing immediate WHCA* replanning.")
        print(f"  Gazebo output: {res.stdout.strip()}")
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to spawn obstacle in Gazebo: {e.stderr.strip()}")
        sys.exit(1)


if __name__ == "__main__":
    main()

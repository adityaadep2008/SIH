#!/usr/bin/env python3
"""Launch Gazebo server + GUI, sequentially spawn 4 AMRs on charging pads, and drive them 3m forward.

Usage:
    ./scripts/launch_four_amrs.py
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
SIH_ROOT = SCRIPT_DIR.parent
WORKSPACE = SIH_ROOT.parent.parent
WAREHOUSE_DIR = WORKSPACE / "src" / "warehouse_world_custom"
WORLD_FILE = WAREHOUSE_DIR / "worlds" / "small_warehouse" / "warehouse_clean.sdf"
GUI_CONFIG = Path("/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config")
CONTROL_CONFIG = SIH_ROOT / "src" / "sih_amr_fleet" / "config" / "fleet_fast_control.yaml"
CYCLONEDDS_XML = SIH_ROOT / "src" / "sih_amr_fleet" / "config" / "cyclonedds.xml"
LOG_BASE_DIR = WORKSPACE / "log" / "four_amr_runs"

# Default charging pad spawn poses (South wall charging bay, facing North +Y / yaw = 1.5708)
CHARGING_PAD_SPAWNS = [
    {"robot_id": "robot_1", "x": -5.25, "y": -29.55, "z": 0.05, "yaw": 1.5708},
    {"robot_id": "robot_2", "x": -3.75, "y": -29.55, "z": 0.05, "yaw": 1.5708},
    {"robot_id": "robot_3", "x": -2.25, "y": -29.55, "z": 0.05, "yaw": 1.5708},
    {"robot_id": "robot_4", "x": -0.75, "y": -29.55, "z": 0.05, "yaw": 1.5708},
]

# Track child processes for clean termination
CHILD_PROCESSES: List[subprocess.Popen] = []
SHUTTING_DOWN = False


def setup_environment():
    """Prepare environment variables for Gazebo and ROS 2 Jazzy."""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "42")
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if CYCLONEDDS_XML.exists():
        env["CYCLONEDDS_URI"] = f"file://{CYCLONEDDS_XML}"
    env["GZ_IP"] = "127.0.0.1"
    env["QT_QPA_PLATFORM"] = "xcb"
    
    current_plugin = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/jazzy/lib:{current_plugin}" if current_plugin else "/opt/ros/jazzy/lib"
    
    gz_resource_paths = [
        str(SIH_ROOT / "src" / "sih_amr_fleet" / "models"),
        str(WAREHOUSE_DIR / "models"),
        str(WAREHOUSE_DIR),
        "/opt/ros/jazzy/share",
    ]
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(gz_resource_paths)
    return env


ENV = setup_environment()


def cleanup(signum=None, frame=None):
    """Terminate all background child processes."""
    global SHUTTING_DOWN
    if SHUTTING_DOWN:
        return
    SHUTTING_DOWN = True
    print("\nStopping four-AMR session and cleaning up processes...")
    
    for proc in reversed(CHILD_PROCESSES):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
    
    time.sleep(1.0)
    
    for proc in reversed(CHILD_PROCESSES):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    print("All processes stopped.")
    sys.exit(0)


def start_process(cmd: List[str], log_path: Path, name: str) -> subprocess.Popen:
    """Start a background process in its own process group with redirected output."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "w")
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        env=ENV,
    )
    CHILD_PROCESSES.append(proc)
    print(f"[{name}] Started (PID: {proc.pid}) -> {log_path.name}")
    return proc


def cleanup_lingering_processes():
    """Clean up any old simulation or AMR processes from prior aborted runs."""
    try:
        subprocess.run(
            ["pkill", "-15", "-f", "gz sim|ros_gz_bridge|spawn_minimal_amr|diffdrive_spawner"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
        subprocess.run(
            ["pkill", "-9", "-f", "gz sim|ros_gz_bridge|spawn_minimal_amr|diffdrive_spawner"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def spawn_amrs(log_dir: Path):
    """Spawn the 4 AMRs sequentially on the charging pads."""
    for i, cfg in enumerate(CHARGING_PAD_SPAWNS, start=1):
        robot_id = cfg["robot_id"]
        x = str(cfg["x"])
        y = str(cfg["y"])
        z = str(cfg["z"])
        yaw = str(cfg["yaw"])
        
        print(f"Spawning {robot_id} at charging pad ({x}, {y}, yaw={yaw})...")
        cmd = [
            "ros2", "launch", "sih_amr_fleet", "spawn_minimal_amr.launch.py",
            f"namespace:={robot_id}",
            "model:=lite",
            "world:=default",
            f"x:={x}",
            f"y:={y}",
            f"z:={z}",
            f"yaw:={yaw}",
            "spawn_dock:=false",
            "keep_sensors_system:=false",
            "sensor_profile:=fleet",
            "lidar_update_rate_hz:=10.0",
            f"control_config:={CONTROL_CONFIG}",
        ]
        start_process(cmd, log_dir / f"{robot_id}.log", f"Spawn {robot_id}")
        
        # Staging delay between sequential AMR launches
        print(f"Waiting 7 seconds before next AMR bring-up...")
        time.sleep(7.0)


def drive_amrs_forward(distance_m: float = 3.0, speed_mps: float = 0.35):
    """Drive all 4 AMRs forward by distance_m meters using cmd_vel commands."""
    print(f"\nAll 4 AMRs spawned! Moving out {distance_m}m from charging pads...")
    
    # Import rclpy if available, or publish using ROS 2 topic / Python publisher loop
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node("drive_out_manager")

        # Track starting positions and distance traveled
        initial_poses: Dict[str, Optional[Tuple[float, float]]] = {
            f"robot_{i}": None for i in range(1, 5)
        }
        current_poses: Dict[str, Optional[Tuple[float, float]]] = {
            f"robot_{i}": None for i in range(1, 5)
        }
        publishers = {
            f"robot_{i}": node.create_publisher(Twist, f"/robot_{i}/cmd_vel", 10)
            for i in range(1, 5)
        }

        def make_odom_callback(robot_name):
            def callback(msg: Odometry):
                pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                current_poses[robot_name] = pos
                if initial_poses[robot_name] is None:
                    initial_poses[robot_name] = pos
            return callback

        for i in range(1, 5):
            r_name = f"robot_{i}"
            node.create_subscription(Odometry, f"/{r_name}/odom", make_odom_callback(r_name), 10)

        # Drive loop
        rate_hz = 10
        dt = 1.0 / rate_hz
        max_duration_s = (distance_m / speed_mps) * 2.0 + 5.0
        start_time = time.time()
        completed = {f"robot_{i}": False for i in range(1, 5)}

        while (time.time() - start_time) < max_duration_s and not all(completed.values()):
            rclpy.spin_once(node, timeout_sec=dt)

            for i in range(1, 5):
                r_name = f"robot_{i}"
                if completed[r_name]:
                    continue

                init_p = initial_poses[r_name]
                curr_p = current_poses[r_name]

                dist = 0.0
                if init_p is not None and curr_p is not None:
                    dist = math.hypot(curr_p[0] - init_p[0], curr_p[1] - init_p[1])

                if dist >= distance_m:
                    # Stop robot
                    stop_twist = Twist()
                    publishers[r_name].publish(stop_twist)
                    completed[r_name] = True
                    print(f"[{r_name}] Reached target distance ({dist:.2f}m >= {distance_m}m). Stopped.")
                else:
                    # Drive forward
                    cmd = Twist()
                    cmd.linear.x = speed_mps
                    publishers[r_name].publish(cmd)

            if all(completed.values()):
                break

        # Send final zero velocity to all
        for r_name, pub in publishers.items():
            pub.publish(Twist())

        print(f"Drive-out complete for all AMRs.\n")
        node.destroy_node()

    except Exception as exc:
        print(f"Notice: Using timed command fallback for driving out: {exc}")
        drive_duration_s = distance_m / speed_mps
        
        # Publish forward cmd_vel
        for _ in range(int(drive_duration_s * 5)):
            for i in range(1, 5):
                subprocess.run(
                    ["ros2", "topic", "pub", "--once", f"/robot_{i}/cmd_vel", "geometry_msgs/msg/Twist",
                     f"{{linear: {{x: {speed_mps}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            time.sleep(0.2)

        # Publish stop
        for i in range(1, 5):
            subprocess.run(
                ["ros2", "topic", "pub", "--once", f"/robot_{i}/cmd_vel", "geometry_msgs/msg/Twist",
                 "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        print(f"Timed drive-out complete.\n")


def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    run_id = time.strftime("%Y%m%d_%H%M%S")
    log_dir = LOG_BASE_DIR / run_id
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"=== Four AMR Bringup Session ===")
    print(f"Logs: {log_dir}")

    # 1. Clean lingering processes
    cleanup_lingering_processes()

    # 2. Start Gazebo Server
    print("Starting Gazebo Server...")
    gz_server_cmd = ["gz", "sim", "-s", "-r", "--render-engine", "ogre2", str(WORLD_FILE)]
    start_process(gz_server_cmd, log_dir / "gazebo_server.log", "Gazebo Server")
    time.sleep(4.0)

    # 3. Start Clock Bridge
    print("Starting ROS-GZ Clock Bridge...")
    clock_cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    start_process(clock_cmd, log_dir / "clock_bridge.log", "Clock Bridge")
    time.sleep(2.0)

    # 4. Start Gazebo GUI
    print("Starting Gazebo GUI...")
    gz_gui_cmd = ["gz", "sim", "-g", "--render-engine", "ogre2", "--gui-config", str(GUI_CONFIG)]
    gui_proc = start_process(gz_gui_cmd, log_dir / "gazebo_gui.log", "Gazebo GUI")
    time.sleep(3.0)

    # 5. Spawn 4 AMRs sequentially
    spawn_amrs(log_dir)

    # 6. Settle briefly then drive 3m forward
    time.sleep(3.0)
    drive_amrs_forward(distance_m=3.0, speed_mps=0.35)

    print("=================================================================")
    print("All 4 AMRs are ready in the warehouse world.")
    print("Keep this terminal open; press Ctrl+C to terminate all processes.")
    print("=================================================================")

    # Keep alive while GUI / Server runs
    try:
        while True:
            if gui_proc.poll() is not None:
                print("Gazebo GUI closed by user.")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    main()

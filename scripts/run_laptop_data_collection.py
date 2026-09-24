#!/usr/bin/env python3
"""Laptop Automated Data Collection Runner (4 AMRs, 0.46 m/s Standard Speed).

Runs sequential work-cycle benchmarks on Laptop (60 FPS headless):
- 4 TurtleBot 4 AMRs (optimal non-congested fleet density)
- Standard 0.46 m/s physical_max velocity profile
- Consolidated robot agent processes
- DDS Domain IDs cycling in [60, 99]
- Use -d to rebuild the complete laptop ML dataset after collection.
"""

import argparse
import datetime
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set

# Terminal Color Styling
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_DIM = "\033[2m"
C_RED = "\033[31m"
C_GREEN = "\033[32m"
C_YELLOW = "\033[33m"
C_BLUE = "\033[34m"
C_MAGENTA = "\033[35m"
C_CYAN = "\033[36m"
C_WHITE = "\033[37m"
C_BG_BLUE = "\033[44m"
C_BG_DARK = "\033[100m"


def collection_log_root() -> Path:
    """Return the shared, portable root for SIH collection artifacts."""
    override = os.environ.get("SIH_DATA_LOG_DIR")
    if override:
        return Path(override).expanduser()
    legacy_root = os.environ.get("AMR_WS_LOG_DIR")
    if legacy_root:
        return Path(legacy_root).expanduser() / "sih_data_collection"
    return Path.home() / "amr_ws" / "log" / "sih_data_collection"

RE_CBBA_BID = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*SUBMIT_BID for task (?P<task>[a-zA-Z0-9_-]+)"
)
RE_CBBA_COMMIT = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*UNANIMOUS_COMMIT for task (?P<task>[a-zA-Z0-9_-]+) -> Winner=(?P<winner>robot_\d),\s*Bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+)\..*Quorum=(?P<quorum>\d+/\d+)"
)
RE_TASK_ACCEPT = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ACCEPT new task (?P<task>[a-zA-Z0-9_-]+)\..*pickup=\((?P<px>[-\d.]+),\s*(?P<py>[-\d.]+)\),\s*dropoff=\((?P<dx>[-\d.]+),\s*(?P<dy>[-\d.]+)\)"
)
RE_PICKUP_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at pickup for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_PICKUP_DONE = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*PICKUP_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)"
)
RE_DROPOFF_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at dropoff for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_TASK_COMPLETED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*(?:DROPOFF_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)\..*Transition to COMPLETED|RESET executor after publishing completion for task (?P<task2>[a-zA-Z0-9_-]+))"
)
RE_ROBOT_READY = re.compile(
    r"(?:\[(?P<robot>robot_\d)\.interface_readiness\]:\s*Interface readiness passed|(?P<robot2>robot_\d) passed all gates)"
)
RE_SPAWN_START = re.compile(
    r"Starting (?P<robot>robot_\d) at x=(?P<x>[-\d.]+) y=(?P<y>[-\d.]+) yaw=(?P<yaw>[-\d.]+)"
)
RE_GZ_SERVER = re.compile(
    r"Starting unthrottled Gazebo server with (?P<engine>\w+)"
)
RE_FLEET_DONE = re.compile(
    r"All (?P<count>\d+) AMRs passed bring-up"
)


class TaskState:
    def __init__(self, task_id: str):
        self.task_id = task_id
        self.assigned_robot: Optional[str] = None
        self.bid: Optional[float] = None
        self.quorum: Optional[str] = None
        self.pickup_coord: Optional[str] = None
        self.dropoff_coord: Optional[str] = None
        self.stage: str = "ANNOUNCED"
        self.start_wall_time: float = time.time()
        self.pickup_time: Optional[float] = None
        self.dropoff_time: Optional[float] = None
        self.completion_time: Optional[float] = None

    @property
    def duration_s(self) -> float:
        if self.completion_time:
            return self.completion_time - self.start_wall_time
        return time.time() - self.start_wall_time


class LaptopCycleRun:
    def __init__(
        self,
        run_index: int,
        total_runs: int,
        target_tasks: int,
        base_dir: Path,
        timeout_s: int,
        fleet_count: int = 4,
        gui: bool = False
    ):
        self.run_index = run_index
        self.total_runs = total_runs
        self.target_tasks = target_tasks
        self.base_dir = base_dir
        self.timeout_s = timeout_s
        self.fleet_count = fleet_count
        self.gui = gui
        self.run_id = f"laptop_run_{run_index:03d}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.log_dir = base_dir / self.run_id
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.fleet_log_file = self.log_dir / "fleet.log"
        self.telemetry_file = self.log_dir / "fleet_telemetry.jsonl"
        self.events_file = self.log_dir / "run_events.jsonl"
        
        self.tasks: Dict[str, TaskState] = {}
        self.completed_tasks: List[str] = []
        self.ready_robots: Set[str] = set()
        self.robot_status: Dict[str, str] = {f"robot_{i}": "IDLE" for i in range(1, self.fleet_count + 1)}
        self.start_time: float = 0.0
        self.end_time: float = 0.0
        self.sim_time_s: float = 0.0
        self.process: Optional[subprocess.Popen] = None
        self.status: str = "PENDING"

    def clean_lingering_processes(self):
        patterns = [
            "gz sim", "gz-sim", "ros_gz_bridge", "parameter_bridge",
            "robot_agent_process", "data_collection_node", "obstacle_spawner_node",
            "warehouse_map_node", "random_task_generator_node", "spawn_minimal_amr",
            "twist_stamper", "localization_node", "interface_readiness",
            "robot_state_publisher", "static_transform_publisher", "diffdrive_spawner",
            "dashboard_bridge_node", "dashboard_bridge", "kinematic_carrier_node",
            "kinematic_carrier", "verify_gazebo_pose", "rmw_zenohd", "zenoh"
        ]
        for pat in patterns:
            try:
                subprocess.run(["pkill", "-15", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(1.0)
        for pat in patterns:
            try:
                subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(0.5)

    def print_stage_event(self, symbol: str, color: str, tag: str, message: str):
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0.0
        timestamp_str = f"[{elapsed:6.1f}s]"
        if sys.stdout.isatty():
            sys.stdout.write("\r\033[K")
        else:
            sys.stdout.write("\r")
        print(f" {C_DIM}{timestamp_str}{C_RESET} {color}{C_BOLD}{symbol} [{tag:<14}]{C_RESET} {message}")
        sys.stdout.flush()

    def format_status_line(self, elapsed: float, rtf: float) -> str:
        parts = []
        for i in range(1, self.fleet_count + 1):
            rid = f"robot_{i}"
            st = self.robot_status.get(rid, "IDLE")
            if st.startswith("TO_PICKUP"):
                c = C_BLUE
            elif st.startswith("DWELL"):
                c = C_YELLOW
            elif st.startswith("TO_DROPOFF"):
                c = C_CYAN
            elif st == "BIDDING":
                c = C_MAGENTA
            else:
                c = C_GREEN
            parts.append(f"R{i}:{c}{st}{C_RESET}")
        robots_str = " ".join(parts)
        pct = int((len(self.completed_tasks) / max(1, self.target_tasks)) * 100)
        return f"\r\033[K {C_DIM}[{elapsed:6.1f}s]{C_RESET} {C_YELLOW}⚡ RTF:{rtf:.2f}x{C_RESET} | {C_BOLD}{C_GREEN}Tasks:{len(self.completed_tasks)}/{self.target_tasks}({pct}%){C_RESET} | {robots_str} "

    def parse_launcher_line(self, line: str):
        line = line.strip()
        if not line:
            return
        
        m = RE_GZ_SERVER.search(line)
        if m:
            self.print_stage_event("⚙", C_YELLOW, "GAZEBO SERVER", f"Starting physics server ({m.group('engine')})...")
            return

        m = RE_SPAWN_START.search(line)
        if m:
            robot = m.group("robot")
            x, y, yaw = m.group("x"), m.group("y"), m.group("yaw")
            self.print_stage_event("⚙", C_BLUE, "SPAWNING AMR", f"Spawning {robot} at ({x}, {y}, yaw={yaw})...")
            return

        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.robot_status[robot] = "IDLE"
                self.print_stage_event("✔", C_CYAN, "GATE PASSED", f"{robot} interfaces ready ({len(self.ready_robots)}/{self.fleet_count})")
            return

        m = RE_FLEET_DONE.search(line)
        if m:
            self.print_stage_event("🚀", C_GREEN, "FLEET READY", f"All {self.fleet_count} AMRs online! Starting task generation & CBBA auction...")
            return

        if "[GAZEBO POSE OK]" in line:
            self.print_stage_event("⚓", C_GREEN, "POSE VERIFIED", line.split("[GAZEBO POSE OK]")[-1].strip())
            return

        if "[GAZEBO POSE MISMATCH]" in line:
            self.print_stage_event("✖", C_RED, "POSE MISMATCH", line.split("[GAZEBO POSE MISMATCH]")[-1].strip())
            return

        if "[GAZEBO POSE TIMEOUT]" in line:
            self.print_stage_event("✖", C_RED, "POSE TIMEOUT", line.split("[GAZEBO POSE TIMEOUT]")[-1].strip())
            return

        if "Verifying" in line and "Gazebo" in line:
            self.print_stage_event("🔍", C_CYAN, "VERIFYING POSE", line.strip())
            return

        if "ERROR:" in line or "fail" in line.lower():
            self.print_stage_event("✖", C_RED, "ERROR", line)

    def parse_log_line(self, line: str):
        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.robot_status[robot] = "IDLE"
                self.print_stage_event("✔", C_CYAN, "ROBOT READY", f"{robot} interfaces ready ({len(self.ready_robots)}/{self.fleet_count})")

        m = RE_CBBA_BID.search(line)
        if m:
            robot = m.group("robot")
            if self.robot_status.get(robot, "IDLE") in ("IDLE", "WAIT_BID", "BIDDING"):
                self.robot_status[robot] = "BIDDING"

        m = RE_CBBA_COMMIT.search(line)
        if m:
            task_id = m.group("task")
            winner = m.group("winner")
            quorum = m.group("quorum")
            bid = float(m.group("bid"))
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "ANNOUNCED":
                t.assigned_robot = winner
                t.bid = bid
                t.quorum = quorum
                t.stage = "CBBA_COMMITTED"
                self.print_stage_event("★", C_MAGENTA, "CBBA DONE", f"Task {C_BOLD}{task_id}{C_RESET} assigned to {C_BOLD}{winner}{C_RESET} (Bid: {bid:.2f}, Quorum: {quorum})")

        m = RE_TASK_ACCEPT.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            px, py = m.group("px"), m.group("py")
            dx, dy = m.group("dx"), m.group("dy")
            self.robot_status[robot_id] = f"TO_PICKUP({task_id})"
            for r in self.robot_status:
                if self.robot_status[r] == "BIDDING":
                    self.robot_status[r] = "IDLE"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ANNOUNCED", "CBBA_COMMITTED"):
                t.assigned_robot = robot_id
                t.pickup_coord = f"({px}, {py})"
                t.dropoff_coord = f"({dx}, {dy})"
                t.stage = "ACCEPTED"
                self.print_stage_event("▶", C_BLUE, "TASK ACCEPT", f"{robot_id} navigating to Pickup {t.pickup_coord} for {task_id}")

        m = RE_PICKUP_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            self.robot_status[robot_id] = f"DWELL_PICKUP({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "ACCEPTED":
                t.stage = "AT_PICKUP"
                t.pickup_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "PICKUP ARRIVED", f"{robot_id} reached pickup station for {task_id} (dwelling {dwell}s)")

        m = RE_PICKUP_DONE.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            self.robot_status[robot_id] = f"TO_DROPOFF({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ACCEPTED", "AT_PICKUP"):
                t.stage = "EN_ROUTE_DROPOFF"
                self.print_stage_event("➜", C_CYAN, "PICKUP DONE", f"{robot_id} finished loading {task_id}; heading to Dropoff {t.dropoff_coord}")

        m = RE_DROPOFF_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            self.robot_status[robot_id] = f"DWELL_DROPOFF({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ACCEPTED", "AT_PICKUP", "EN_ROUTE_DROPOFF"):
                t.stage = "AT_DROPOFF"
                t.dropoff_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "DROPOFF ARRIVED", f"{robot_id} reached dropoff station for {task_id} (dwelling {dwell}s)")

        m = RE_TASK_COMPLETED.search(line)
        if m:
            task_id = m.group("task") or m.group("task2")
            robot_id = m.group("robot")
            if robot_id:
                self.robot_status[robot_id] = "IDLE"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage != "COMPLETED":
                t.stage = "COMPLETED"
                t.completion_time = time.time()
                if task_id not in self.completed_tasks:
                    self.completed_tasks.append(task_id)
                    count = len(self.completed_tasks)
                    pct = int((count / self.target_tasks) * 100)
                    bar = "█" * (pct // 2) + "░" * (50 - (pct // 2))
                    self.print_stage_event(
                        "✔", C_GREEN, "TASK COMPLETED",
                        f"{C_BOLD}{task_id}{C_RESET} finished by {t.assigned_robot} | Duration: {t.duration_s:.1f}s | Progress: [{bar}] {count}/{self.target_tasks} ({pct}%)"
                    )

    def execute(self, tracking_speed: float = 0.46, settle_s: int = 3, seed: int = 2000) -> bool:
        self.clean_lingering_processes()
        self.start_time = time.time()
        
        env = os.environ.copy()
        env["START_GUI"] = "true" if self.gui else "false"
        env["START_FLEET"] = "true"
        env["FLEET_RANDOM_TASKS"] = "true"
        env["FLEET_RECORD_DATA"] = "true"
        env["FLEET_COUNT"] = str(self.fleet_count)
        env["RENDER_ENGINE"] = "ogre2"
        env["SENSOR_PROFILE"] = "fleet"
        env["LIDAR_UPDATE_RATE_HZ"] = "5.0"
        env["FLEET_TRACKING_SPEED_MPS"] = str(tracking_speed)
        env["SETTLE_SECONDS"] = str(settle_s)
        env["LOG_DIR"] = str(self.log_dir)
        env["FLEET_DATA_FILE"] = str(self.telemetry_file)
        env["FLEET_RANDOM_SEED"] = str(seed + self.run_index)
        env["FLEET_ENABLE_FAULTS"] = "false"
        env["FLEET_ENABLE_SPAWNER"] = "true"
        env["FLEET_ENABLE_VISION"] = "false"
        env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        env["CUDA_VISIBLE_DEVICES"] = "0"
        
        # Laptop domain range [60, 99] - zero collision with desktop [10, 49]
        run_domain_id = 60 + (self.run_index % 40)
        env["ROS_DOMAIN_ID"] = str(run_domain_id)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

        script_dir = Path(__file__).resolve().parent
        launch_script = script_dir / "launch_fleet_amrs.sh"
        if not launch_script.exists():
            print(f"{C_RED}ERROR: launch_fleet_amrs.sh not found at {launch_script}{C_RESET}")
            return False

        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING LAPTOP TEST RUN {self.run_index}/{self.total_runs}: {self.run_id} <<<{C_RESET}")
        print(f" {C_CYAN}Target Work Cycle:{C_RESET} {self.target_tasks} completed tasks across {self.fleet_count} AMRs")
        print(f" {C_CYAN}Log Directory:{C_RESET}     {self.log_dir}")
        print(f" {C_CYAN}DDS Domain ID:{C_RESET}     {run_domain_id} (Range [60, 99]) | Seed: {seed + self.run_index}")
        print(f" {C_CYAN}Fleet Architecture:{C_RESET}Consolidated Agent Processes, 50 Hz Physics, Lean Telemetry\n")
        sys.stdout.flush()

        self.process = subprocess.Popen(
            ["bash", str(launch_script)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1
        )

        output_queue = queue.Queue()

        def stream_reader():
            try:
                for line in iter(self.process.stdout.readline, ''):
                    output_queue.put(line)
                self.process.stdout.close()
            except Exception:
                pass

        reader_thread = threading.Thread(target=stream_reader, daemon=True)
        reader_thread.start()

        fleet_log_fd = None
        telemetry_fd = None
        last_ticker_s = -1
        first_sim_s = None
        
        try:
            while self.process.poll() is None:
                # Read stdout from launcher script
                while not output_queue.empty():
                    try:
                        lline = output_queue.get_nowait()
                        self.parse_launcher_line(lline)
                    except queue.Empty:
                        break

                if len(self.completed_tasks) >= self.target_tasks:
                    self.status = "PASSED"
                    break

                elapsed = time.time() - self.start_time
                if elapsed > self.timeout_s:
                    self.status = "TIMEOUT"
                    break

                # Tail fleet.log for consensus and executor events
                if fleet_log_fd is None and self.fleet_log_file.exists():
                    fleet_log_fd = open(self.fleet_log_file, "r", encoding="utf-8", errors="ignore")

                if fleet_log_fd is not None:
                    for line in fleet_log_fd:
                        self.parse_log_line(line)

                # Tail telemetry file incrementally
                if telemetry_fd is None and self.telemetry_file.exists():
                    telemetry_fd = open(self.telemetry_file, "r", encoding="utf-8", errors="ignore")

                if telemetry_fd is not None:
                    for tline in telemetry_fd:
                        if tline.strip():
                            try:
                                rec = json.loads(tline)
                                sim_t = float(rec.get("logged_at", 0.0))
                                if sim_t > 0.0:
                                    if first_sim_s is None:
                                        first_sim_s = sim_t
                                    self.sim_time_s = sim_t
                            except Exception:
                                pass

                # Update live ticker and AMR status line every 1s
                now_s = int(elapsed)
                if now_s != last_ticker_s:
                    last_ticker_s = now_s
                    rtf = ((self.sim_time_s - first_sim_s) / max(1.0, elapsed)) if (first_sim_s and self.sim_time_s > first_sim_s) else 0.0
                    sys.stdout.write(self.format_status_line(elapsed, rtf))
                    sys.stdout.flush()

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.status = "ABORTED"
        finally:
            if fleet_log_fd:
                fleet_log_fd.close()
            if telemetry_fd:
                telemetry_fd.close()
            sys.stdout.write("\n")
            sys.stdout.flush()
            self.end_time = time.time()
            if self.process and self.process.poll() is None:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                    self.process.wait(timeout=10)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except Exception:
                        pass
            self.clean_lingering_processes()

        wall_duration = self.end_time - self.start_time
        rtf = (self.sim_time_s / wall_duration) if wall_duration > 0 else 0.0
        print(f"\n\n [Run {self.run_index} Summary] Status: {self.status} | Completed Tasks: {len(self.completed_tasks)}/{self.target_tasks} | Wall Time: {wall_duration:.1f}s | RTF: {rtf:.2f}x\n")
        return self.status == "PASSED"


def main():
    parser = argparse.ArgumentParser(
        description="Laptop Fleet Data Collection Runner (4 AMRs, 0.46 m/s Standard Speed)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-r", "--runs", type=int, default=1, help="Number of simulation work cycles / runs")
    parser.add_argument("-t", "--tasks", type=int, default=12, help="Target completed tasks per cycle")
    parser.add_argument("-s", "--speed", type=float, default=0.46, help="AMR path tracking speed m/s (default: 0.46 m/s physical_max)")
    parser.add_argument("-f", "--fleet-size", type=int, default=4, help="Fleet AMR count")
    parser.add_argument("--seed", type=int, default=2000, help="Base random seed for Laptop")
    parser.add_argument("--timeout", type=int, default=3600, help="Per-run timeout seconds")
    parser.add_argument("--gui", action="store_true", default=False, help="Launch Gazebo with GUI enabled (default: headless)")
    repo_root = Path(__file__).resolve().parent.parent
    parser.add_argument("-d", "--compile-dataset", action="store_true", help="Rebuild the complete laptop dataset from all saved laptop telemetry after the run")
    parser.add_argument("-o", "--output-csv", default=str(repo_root / "collected_datasets_laptop.csv"), help="Dataset output path used with -d")
    args = parser.parse_args()

    log_root = collection_log_root()
    base_dir = log_root / f"laptop_data_collection_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    base_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{C_BOLD}{C_GREEN}======================================================================{C_RESET}")
    print(f"{C_BOLD}{C_GREEN}  SIH LAPTOP AUTOMATED DATA COLLECTION: {args.fleet_size} AMRs, {args.runs} RUN(S) × {args.tasks} TASKS  {C_RESET}")
    print(f"{C_BOLD}{C_GREEN}  TARGET: {args.runs * args.tasks} DATASET TASKS @ {args.speed} m/s FOR ML CONGESTION MODEL  {C_RESET}")
    print(f"{C_BOLD}{C_GREEN}======================================================================{C_RESET}\n")

    telemetry_files = []
    passed = 0

    try:
        for idx in range(1, args.runs + 1):
            run = LaptopCycleRun(
                idx, args.runs, args.tasks, base_dir, args.timeout,
                fleet_count=args.fleet_size, gui=args.gui
            )
            success = run.execute(tracking_speed=args.speed, settle_s=3, seed=args.seed)
            if run.telemetry_file.exists() and run.telemetry_file.stat().st_size > 0:
                telemetry_files.append(str(run.telemetry_file))
            if success:
                passed += 1
            if run.status == "ABORTED":
                print(f"\n{C_YELLOW}Run {idx} was aborted. Halting remaining runs.{C_RESET}")
                break
    except KeyboardInterrupt:
        print(f"\n{C_YELLOW}Interrupted by user. Halting simulation runs.{C_RESET}")

    if args.compile_dataset:
        telemetry_pattern = str(log_root / "laptop_data_collection_*" / "**" / "fleet_telemetry.jsonl")
        print(f"\n{C_BOLD}{C_CYAN}>>> Rebuilding laptop ML dataset from all saved laptop telemetry: {args.output_csv} <<<{C_RESET}")
        script_dir = Path(__file__).resolve().parent
        gen_script = script_dir / "generate_ml_dataset.py"
        if gen_script.exists():
            result = subprocess.run(["python3", str(gen_script), telemetry_pattern, "--output", args.output_csv])
            if result.returncode == 0 and Path(args.output_csv).exists():
                print(f"\n{C_BOLD}{C_GREEN}✔ Laptop dataset rebuilt from all saved logs: {args.output_csv}{C_RESET}\n")
            elif result.returncode != 0:
                print(f"\n{C_RED}Dataset compilation failed with exit code {result.returncode}.{C_RESET}\n")
    else:
        print(f"\n{C_GREEN}✔ Laptop Data Collection Finished: {passed}/{args.runs} runs completed. Use -d to rebuild the full laptop dataset.{C_RESET}\n")


if __name__ == "__main__":
    main()

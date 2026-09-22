# Mandatory AI Agent Operational Governance Rules & Fleet Guide

> **CRITICAL REPOSITORY PROTECTION POLICY**:
> This file establishes strict, immutable boundaries for all AI coding assistants (including Antigravity, Gemini, and any subagents) operating on this repository. Every agent must strictly adhere to these rules without exception.

---

## 1. Absolute Prohibition: No Agent-Driven Final Script Execution

### Rule
**NO AI AGENT SHALL EVER RUN FINAL DATA COLLECTION, FLEET BENCHMARKING, OR LONG-RUNNING SIMULATION SCRIPTS.**

This includes, but is not limited to:
- `scripts/run_desktop_data_collection.py`
- `scripts/run_laptop_data_collection.py`
- `scripts/run_multi_work_cycles.py`
- `scripts/run_speed_and_fleet_optimization.py`
- `scripts/launch_fleet_amrs.sh`
- `scripts/launch_four_amrs.sh`
- `scripts/launch_four_lite.sh`
- `scripts/launch_four_standard.sh`
- `scripts/warehouse.sh`
- Any command that launches Gazebo simulation, spawns multi-robot fleets, or runs live work cycles.

**Execution of these scripts is reserved EXCLUSIVELY for human operators.**

### What Agents CAN Run:
- Clean build commands (e.g. `colcon build --packages-select <pkg>`)
- Discrete unit tests (e.g. `pytest src/...`, `colcon test`)
- Read-only diagnostics (inspecting logs, reviewing telemetry datasets, checking git status/diffs, verifying process tables with `ps`/`pgrep`)
- Read-only static analysis and syntax checks

---

## 2. Mandatory Forensic Diagnosis Before Any Proposed Changes

### Rule
**NO CODE CHANGES SHALL BE MADE WITHOUT FORENSIC INVESTIGATION AND IRREFUTABLE PROOF.**

Before touching any code:
1. **Analyze Past Runs & Logs**: Agents must thoroughly examine past run artifacts, telemetry logs (`fleet_telemetry.jsonl`), run events (`run_events.jsonl`), carrier logs, Gazebo server logs, and system metrics.
2. **Diagnose Beyond a Doubt**: Agents must identify the root cause with empirical evidence (timestamps, coordinates, error messages, process IDs, mathematical calculations).
3. **Present a Solid Forensic Case**: Agents must clearly present their diagnosis and proof to the human operator before formulating a fix.

---

## 3. Strict Human Verification Gate (Plan-Only First)

### Rule
**AGENTS MUST ONLY PRESENT A PLAN TO THE HUMAN. NO CODE MODIFICATION MAY OCCUR WITHOUT EXPLICIT HUMAN APPROVAL.**

1. Once the forensic diagnosis is verified, agents must prepare and present a detailed implementation plan.
2. The implementation plan must detail the exact files, mathematical rationale, parameter values, and unit verification steps.
3. **STOP AND WAIT**: The agent MUST NOT modify code, create commits, or execute alterations until the human explicitly reviews and verifies the plan with an affirmative instruction to proceed.

---

## 4. Absolute Prohibition: No Tampering With Proven Core Baseline Algorithms

### Rule
**NO AI AGENT SHALL EVER ALTER, HACK, OVERRIDE, OR DEPRECATE PROVEN CORE BASELINE COORDINATION ARCHITECTURE.**

The verified baseline algorithms that achieved 20/20 flawless task completion (WHCA* 4D space-time reservations, reciprocal ORCA velocity obstacles, CBBA decentralized auction consensus, and Corridor Mutex single-aisle locking) are immutable foundational assets.

**Specifically, AI AGENTS ARE STRICTLY FORBIDDEN FROM:**
1. **Ad-Hoc Corridor Penalties**: Adding artificial corridor lane penalties, directional costs, or heuristic distortions into backward Dijkstra or forward A* search (`whca_star`). WHCA* 4D space-time coordination must resolve corridor traffic without artificial single-lane funnels.
2. **Asymmetric Yielding & Velocity Clamping**: Introducing asymmetric robot-ID priority yields, hard overrides, or artificial zero-velocity clamps into reciprocal velocity obstacles (`avoidance_velocity` / `orca_node.py`). Reciprocal velocity-obstacle mathematics must remain balanced and uncorrupted.
3. **Static All-Horizon Peer Blockages**: Injecting synthetic permanent obstacles across future horizon time steps in `whca_planner_node.py`. Space-time dynamic reservations must coordinate trailing and moving AMRs naturally.
4. **Architectural Hacking**: Rewriting or replacing established coordination components (`CBBA`, `TaskExecutor`, `CorridorMutex`, `SafetySupervisor`) based on loose interpretations or speculative 'enhancements'.
5. **Regression Verification Requirement**: Any approved bug fix MUST be accompanied by a regression unit test demonstrating that baseline throughput and zero-collision guarantees are preserved.

---

## Summary Directive for AI Agents
- **Do not run simulation/benchmark scripts.**
- **Diagnose with concrete evidence first.**
- **Present a plan and wait for human sign-off.**
- **Never tamper with core baseline coordination algorithms.**
- **Build and unit-test only after approval.**

---

# Technical Architecture & Reference Guide

## System & Environment Specifications

- **Operating System**: Ubuntu 24.04.4 LTS (Noble Numbat) x86_64 (`Linux 7.0.0-31-generic`)
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
- **Gazebo Simulator**: Gazebo Sim Harmonic (version 8.11.0, `gz sim`)
- **Python Version**: Python 3.12.3
- **DDS Middleware**: Fast DDS with UDPv4 transport (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `ROS_DOMAIN_ID=42`)
- **Workspace Layout**:
  - Workspace Root: `/home/rtsws/amr_ws`
  - SIH Project Repository: `/home/rtsws/amr_ws/src/SIH`
  - Build & Install Target: `/home/rtsws/amr_ws/build` and `/home/rtsws/amr_ws/install`

---

## Active Shell Configuration & Aliases

The current active environment is configured in `~/.bashrc`.

### ROS 2 & Fast DDS Environment
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

### Registered Bash Aliases

| Alias | Command / Target | Purpose |
|---|---|---|
| `warehouse` | `bash "$HOME/amr_ws/src/SIH/scripts/warehouse.sh"` | Launches Gazebo warehouse world with proper resource paths & renderer |
| `bridge_clock` | `source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"` | Starts clock bridge between Gazebo and ROS 2 |
| `spawn_turtlebot` | `source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 launch sih_amr_fleet spawn_robot_1.launch.py namespace:=robot_1 model:=standard x:=2.0 y:=2.0 z:=0.05 yaw:=0.0` | Spawns a single TurtleBot 4 (Standard) for `robot_1` |
| `start_charging` | `source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 run sih_amr_fleet charging_pad_node --ros-args -r __ns:=/robot_1 -p initial_battery_percent:=50.0` | Runs battery & charging station monitor for `robot_1` |
| `amr4` | `bash "$HOME/amr_ws/src/SIH/scripts/launch_four_lite.sh"` | Launches full 4-AMR fleet simulation (Lite model, Ogre2 renderer) |
| `amr4_standard` | `bash "$HOME/amr_ws/src/SIH/scripts/launch_four_standard.sh"` | Launches full 4-AMR fleet simulation (Standard model) |
| `amr4_headless` | `START_GUI=false bash "$HOME/amr_ws/src/SIH/scripts/launch_four_lite.sh"` | Headless 4-AMR simulation (no GUI overhead, optimal for data collection/benchmarks) |
| `launch_four_lite` | `bash "$HOME/amr_ws/src/SIH/scripts/launch_four_lite.sh"` | Direct alias for 4-AMR Lite launch script |
| `launch_four_standard` | `bash "$HOME/amr_ws/src/SIH/scripts/launch_four_standard.sh"` | Direct alias for 4-AMR Standard launch script |
| `gzogre` | `gz sim --render-engine ogre` | Launches Gazebo with Ogre 1 rendering engine |
| `gzogre2` | `gz sim --render-engine ogre2` | Launches Gazebo with Ogre 2 rendering engine |
| `ll` / `la` / `l` | `ls -alF` / `ls -A` / `ls -CF` | Directory listing shortcuts |
| `alert` | Desktop notification helper | Sends notification upon task completion |

---

## Workspace & Repository Architecture

### Core Packages (`src/`)

```text
/home/rtsws/amr_ws/src/SIH/
├── src/
│   ├── sih_amr_interfaces/      # Custom ROS 2 IDL message & service definitions
│   │   ├── msg/                 # RobotState, Task, Bid, TrajectoryIntent, CorridorReservation, etc.
│   │   └── srv/                 # Mutual exclusion and coordination service definitions
│   └── sih_amr_fleet/           # Core fleet management, planning & simulation nodes (Python / rclpy)
│       ├── config/              # Layouts, costmaps, simulation configs, logging YAML
│       ├── launch/              # Modular ROS 2 launch files (fleet, spawn, simulation)
│       ├── maps/                # Grid maps & occupancy representations
│       ├── models/              # Custom charging pads, static assets
│       ├── sih_amr_fleet/       # Node implementations (CBBA, WHCA*, ORCA, Safety, etc.)
│       └── test/                # Unit and integration tests
├── scripts/                     # Automation scripts, launch scripts, dataset generators
├── frontend/                    # Web-based fleet visualization & management dashboard
└── warehouse_layout.lock.yaml   # Canonical warehouse geometry & coordinate definitions
```

---

## Decentralized Multi-AMR Fleet Architecture

All AMRs run decentralized nodes under their own namespaces (`/robot_1`, `/robot_2`, `/robot_3`, `/robot_4`). Inter-robot coordination occurs via distributed consensus over shared `/fleet/*` topics without a centralized single point of failure.

### Per-Robot Node Responsibilities

| Node | Responsibility |
|---|---|
| `localization_node` | Ingests simulator wheel odometry / Gazebo ground truth and publishes validated `RobotState` |
| `local_costmap_node` | Processes 2D LiDAR `scan` into local occupancy grids and obstacle proximity maps |
| `blockage_detector_node` | Identifies persistent pathway blockages and shares TTL-bound map updates with peers |
| `peer_tracker_node` | Tracks peer positions, velocities, and predicts short-horizon trajectories with uncertainty bounds |
| `health_node` | Monitors battery levels, hardware status, and safety state |
| `charging_pad_node` | Simulates physical docking alignment and battery recharge cycles on designated pads |
| `cbba_node` | Consensus-Based Bundle Algorithm for distributed, multi-agent task bidding and allocation |
| `whca_planner_node` | Windowed Hierarchical Cooperative A* for time-space collision-free trajectory generation |
| `reservation_manager_node`| Publishes and synchronizes space-time trajectory reservations on `/fleet/trajectory_intent` |
| `corridor_mutex_node` | Distributed Ricart–Agrawala style mutual exclusion for single-lane aisle navigation |
| `path_follower_node` | Waypoint-following velocity generator (`cmd_vel`) tracking planned trajectories |
| `orca_node` | Optimal Reciprocal Collision Avoidance for real-time dynamic obstacle and peer avoidance |
| `safety_supervisor_node` | Deterministic safety arbiter with emergency stop override before sending velocities to hardware |
| `task_scenario_node` | Generates reproducible task scenarios and benchmarks across the fleet |
| `dashboard_bridge_node` | Aggregates JSON telemetry on `/fleet/dashboard_telemetry` for web UI display |

---

## Standard Build & Execution Workflows

### 1. Building the Workspace
From the workspace root:
```bash
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launching 4-AMR Fleet Simulation (HUMAN OPERATOR EXCLUSIVE)
*(AI Agents: DO NOT execute these commands. See Rule 1.)*
```bash
# Terminal 1: Full simulation with GUI
amr4

# Or for headless execution (e.g. benchmarking / data collection):
amr4_headless
```

### 3. Running Data Collection & Optimization Scripts (HUMAN OPERATOR EXCLUSIVE)
*(AI Agents: DO NOT execute these commands. See Rule 1.)*
```bash
cd ~/amr_ws/src/SIH

# Execute multi-work cycle benchmark
python3 scripts/run_multi_work_cycles.py

# Run fleet speed & coordination optimization
python3 scripts/run_speed_and_fleet_optimization.py
```

### 4. Process Cleanup & DDS Diagnostics
If lingering simulation processes or DDS lock errors occur:
```bash
pkill -9 -f 'gz sim|ros_gz|turtlebot4|robot_state_publisher|parameter_bridge'
ros2 daemon stop
ros2 daemon start
```

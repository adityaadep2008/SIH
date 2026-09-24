# SIH AMR Warehouse Simulation

This repository is the decentralized ROS 2 Jazzy multi-AMR fleet overlay for the custom Gazebo Harmonic warehouse at `~/amr_ws/src/warehouse_world_custom`.

- **Primary Communication**: Eclipse Zenoh (`rmw_zenoh_cpp`) with centralized localhost router daemon (`rmw_zenohd`), eliminating $O(N^2)$ discovery storms and cutting framing overhead by ~75%.
- **Deterministic Backup**: Cyclone DDS (`rmw_cyclonedds_cpp`), fully preserved and selectable via `--dds`.
- **Coordination & Planning**: Conflict-Oriented Windowed Hierarchical Cooperative A* (CO-WHCA*) with dynamic conflict resolvability, CBBA consensus, Ricart-Agrawala corridor mutex, and reciprocal ORCA velocity obstacles.

## Current Verified Baseline

The fleet baseline has achieved verified end-to-end task completion with 100% mission success across sustained multi-AMR runs. Gazebo operates as a high-fidelity kinematic LiDAR renderer that closely tracks the AMRs' internally estimated map positions, decoupling heavy wheel-contact physics while preserving realistic ray-traced sensing, dynamic obstacle detection, and strict safety validation.

### End-to-End Validation Results (40/40 Tasks Complete)

The fleet completed two consecutive 20-task end-to-end benchmark runs (40 completed tasks total) under the standard 0.46 m/s speed profile with zero localization invalidations, zero tilt incidents, and zero accumulating drift.

| Measurement | Run 1 (20 tasks) | Run 2 (20 tasks) | Combined (40 tasks) |
|---|---|---|---|
| **Completed Tasks** | **20 / 20 (100%)** | **20 / 20 (100%)** | **40 / 40 (100%)** |
| **Mean Map ↔ Gazebo Error** | 1.88 cm | 1.81 cm | **1.84 cm** |
| **Maximum Pose Discrepancy** | 24.81 cm | 22.32 cm | 24.81 cm |
| **Samples within 5 cm** | 97.07% | 96.77% | **96.92%** |
| **Samples within 10 cm** | 99.62% | 99.67% | **99.64%** |
| **Invalid Localization Samples** | 0 | 0 | **0** |
| **Robot Tilt Incidents** | 0 | 0 | **0** |
| **Scan-Stale Safety Stops (Run 2)** | — | 0 | **0** |

### Key Benchmark Findings
- **Zero Accumulating Drift**: All 38,519 recorded pose samples remained within the map's 35 cm tolerance. Breaking each run into quarters, the mean discrepancy remained consistently between 1.68 cm and 2.05 cm across all quarters. Final terminal discrepancies were at most 1.85 cm in Run 1 and 1.12 cm in Run 2.
- **Transient Latency vs. Drift**: The occasional 20–25 cm peak discrepancies were transient Gazebo pose-update transport latencies during fast turns, not permanent divergence.
- **Timing & Rates**: The internal kinematic carrier integration operates at **50 Hz**, while Gazebo LiDAR operates at **5 Hz**. Every AMR passed both odometry and scan readiness gates.
- **Sensing Alignment**: Gazebo LiDAR directly represents physical obstacles around the exact coordinate where each AMR internally believes it is located.

---

## Fleet Speed and Footprint Limits

- **Default Standard Speed**: **0.46 m/s** (`physical_max` profile), matching the physical TurtleBot 4 differential-drive hardware limit.
- **Certified Operating Profiles**:
  - `physical_fidelity`: 0.31 m/s nominal, 0.46 m/s max (strict physical fidelity).
  - `physical_max`: 0.46 m/s nominal and max (standard production default).
- **Footprint**: 0.56 m diameter planning footprint (0.28 m radius). The narrow centre aisles are 1.1554 m wide, providing safe clearance for bidirectional passing corridors.
- **Time-Space Slots**: WHCA* derives each space-time reservation slot duration dynamically from the configured tracking speed (`slot_duration = 0.5 m / speed`). At 0.46 m/s, each grid move maps to ~1.087 seconds.
- **Conflict-Oriented Prioritization**: CO-WHCA* calculates a dynamic Conflict Resolvability Index for every AMR, automatically prioritizing boxed-in or mutex-holding AMRs to prevent narrow-aisle gridlock.

---

## Architecture Overview

```text
+-----------------------------------------------------------------------------------+
|                            DECENTRALIZED FLEET NODES                              |
|                                                                                   |
|  [Random Task Generator] ---> [/fleet/task_announcement]                          |
|                                      |                                            |
|                                      v                                            |
|  [cbba_node] <-----------------> Two-Phase Consensus (BID / CLAIM Quorum)         |
|        |                                                                          |
|        v (local task_assignment)                                                  |
|  [task_execution_node] -------> EN_ROUTE_PICKUP -> PICKUP_DWELL ->                |
|        |                        EN_ROUTE_DROPOFF -> DROPOFF_DWELL -> COMPLETED    |
|        v                                                                          |
|  [whca_planner_node] <--------> Space-Time A* (12-slot rolling window)            |
|        |                        + Trajectory Intent Reservations                  |
|        v                                                                          |
|  [corridor_mutex_node] <------> Ricart-Agrawala Lamport Mutex (Narrow Aisles)     |
|        |                                                                          |
|        v                                                                          |
|  [path_follower_node] --------> Waypoint Guidance + Deceleration Ramps           |
|        |                                                                          |
|        v                                                                          |
|  [orca_node] -----------------> Reciprocal Velocity Obstacle Local Avoidance     |
|        |                                                                          |
|        v                                                                          |
|  [safety_supervisor_node] ----> Hard Braking Envelope + LiDAR Directional Guard   |
|        |                                                                          |
|        v (/robot_N/cmd_vel)                                                       |
+--------|--------------------------------------------------------------------------+
         |
         v
+-----------------------------------------------------------------------------------+
|                     KINEMATIC CARRIER & GAZEBO SIMULATION                         |
|                                                                                   |
|  [kinematic_carrier_node] (50 Hz)                                                 |
|    - Integrates planar kinematics from /robot_N/cmd_vel                           |
|    - Enforces swept circular footprint collision against shelves & peer AMRs      |
|    - Publishes simulated wheel odometry (/robot_N/odom)                           |
|    - Dispatches batch entity poses via gz.transport13 (/set_pose_vector)          |
|                                                                                   |
|  [Gazebo Harmonic Server] (5 Hz LiDAR Ray-Tracing)                                |
|    - Positions lightweight carrier models at exact kinematic coordinates          |
|    - Renders 2D LiDAR scans published to ROS 2 (/robot_N/scan)                    |
+-----------------------------------------------------------------------------------+
```

---

## One-Time Build & Setup

```bash
source /opt/ros/jazzy/setup.bash
cd ~/amr_ws
colcon build --symlink-install --packages-select sih_amr_interfaces sih_amr_fleet
source install/setup.bash
```

Create a local pose file from the tracked, verified example:

```bash
mkdir -p ~/.config
cp ~/amr_ws/src/SIH/scripts/sih_amr_poses.env.example \
  ~/.config/sih_amr_poses.env
```

Defaults place robots 1–4 on charging pads 1–4 respectively along the south-wall charging bay.

---

## Running the Fleet

### 1. Automated Benchmark & Data Collection (Default: Zenoh)
To run automated work cycles with live terminal progress, real-time metrics, and automatic CSV/JSONL dataset export at the 0.46 m/s standard speed:

```bash
# Runs with Eclipse Zenoh (rmw_zenoh_cpp) by default:
python3 ~/amr_ws/src/SIH/scripts/run_desktop_data_collection.py -r 2 -t 20

# Run with Cyclone DDS backup using the --dds flag:
python3 ~/amr_ws/src/SIH/scripts/run_desktop_data_collection.py --dds -r 2 -t 20

# Laptop collection:
python3 ~/amr_ws/src/SIH/scripts/run_laptop_data_collection.py -r 2 -t 20
```

### 2. Full Multi-AMR Baseline Launcher
To start Gazebo, the 50 Hz kinematic carrier backend, four AMRs, all decentralized fleet nodes, random tasks, and passive telemetry in a single session:

```bash
bash ~/amr_ws/src/SIH/scripts/run_baseline_random_fleet.sh
```

To activate the Zenoh shell environment in any standalone terminal:
```bash
source ~/amr_ws/src/SIH/scripts/env_zenoh.sh
```

To stop the entire run cleanly, press `Ctrl+C` in that terminal.

### 3. Interactive Shell Aliases
Add aliases to your shell:

```bash
alias_source='source "$HOME/amr_ws/src/SIH/scripts/sih_amr_aliases.sh"'
grep -qxF "$alias_source" ~/.bashrc || printf '\n%s\n' "$alias_source" >> ~/.bashrc
source ~/.bashrc
```

Available commands:
- `amr4`: Starts 4-AMR baseline, clock bridge, sequential spawn gates, and attaches the Gazebo GUI.
- `amr4_headless`: Starts headless 4-AMR baseline for sustained data collection.
- `amr4_standard`: Uses full TurtleBot 4 Standard mesh representation.

---

## Core Algorithmic Subsystems

### 1. Consensus-Based Auction Algorithm (CBBA)
- **Two-Phase Quorum**: AMRs broadcast immutable `BID`s upon task announcement. After a collection window, each replica independently determines the winning tuple `(bid, winner_id)` and broadcasts a signed `CLAIM`.
- **Unanimous Quorum**: The winning AMR only assigns the task to its local executor after receiving matching claims from all active fleet members.
- **Immutability**: Bids are frozen per auction epoch, preventing asynchronous state changes from breaking consensus quorum.

### 2. Conflict-Oriented Space-Time WHCA* (CO-WHCA*)
- **Rolling Horizon**: Searches `(x, y, time_slot)` over a 12-slot rolling window on a 0.5 m resolution occupancy grid.
- **Reverse BFS Heuristic**: Guarantees liveness around shelf obstacles without getting trapped in local minima.
- **Conflict Resolvability Index**: Dynamically scores each AMR's ability to safely yield or retreat based on rear clearance, lateral branching cells, and corridor mutex ownership.
- **Anti-Deadlock Priority Inversion**: Elevates boxed-in AMRs (rear clearance $< 2.5\text{ m}$) and mutex holders to Priority 150, forcing unconstrained AMRs in open space (Priority 100) to yield. This eliminates the classic "Boxed-In Yielder" trap.
- **Space-Time Conflict Detection**: Explicitly identifies vertex collisions $(x, y, t)$ and edge swaps $(t \leftrightarrow t+1)$, injecting localized temporal constraints only around winning trajectory segments.
- **Dynamic Reservations**: Emits trajectory intents on `/fleet/trajectory_intent` to reserve space-time cells and avoid inter-robot collisions strategically.
- **Semantic Blockage Filtering**: Distinguishes transient dynamic obstacles from static shelves and peer silhouettes, filtering out false positive blockages.

### 3. Distributed Corridor Mutex
- **Ricart–Agrawala Algorithm**: Manages single-lane narrow aisles via Lamport logical timestamps.
- **Authority**: Entry requires unanimous grant from healthy peers plus an affirmative `entrance_clear` from the local Safety Supervisor.

### 4. Authoritative Safety Supervisor
- **Hard Braking Envelope**: Continuously monitors directional LiDAR clearance: `d_stop = v^2 / (2 * a_decel) + d_margin`.
- **Preemptive Gating**: Commands immediate `STOP` on sensor staleness, communication loss, or boundary incursions.
- **Non-overridable**: Planners, velocity obstacles (ORCA), and external controllers cannot bypass safety supervisor commands.

### 5. Eclipse Zenoh Middleware Layer (`rmw_zenoh_cpp`)
- **Zero Discovery Storms**: Linear $O(N)$ registration via local `rmw_zenohd` daemon replaces quadratic $O(N^2)$ multicast discovery storms.
- **Ultra-Compact Framing**: 4–6 byte wire headers slash bandwidth consumption on high-rate coordination topics by ~75% compared to DDS RTPS (24–40 bytes).
- **Dual-Stack Reliability**: Defaults to Zenoh across all launchers, with Cyclone DDS fully preserved and instantly accessible via `--dds`.

---

## Repository Structure

```text
SIH/
├── CURRENT_IMPLEMENTATION_GUIDE.md    # In-depth architectural & algorithmic reference
├── README.md                          # Repository overview & quickstart
├── scripts/
│   ├── env_zenoh.sh                   # Zenoh shell environment activator
│   ├── run_desktop_data_collection.py # Automated multi-cycle data collector (Zenoh default, --dds backup)
│   ├── run_laptop_data_collection.py  # Laptop automated data collector (Zenoh default, --dds backup)
│   ├── run_baseline_random_fleet.sh   # Full baseline launcher
│   ├── launch_fleet_amrs.sh           # Core launch & spawn orchestrator (rmw_zenohd lifecycle)
│   ├── verify_gazebo_pose.py          # Ground-truth pose & tilt validator
│   └── sih_amr_aliases.sh             # Interactive shell shortcuts
└── src/
    ├── sih_amr_interfaces/            # ROS 2 custom message & service definitions
    └── sih_amr_fleet/                 # Python package containing all fleet nodes
        ├── config/
        │   ├── zenoh_session_config.json5 # Production Zenoh session parameters
        │   └── cyclonedds.xml             # Preserved Cyclone DDS configuration
        ├── cbba_node.py               # Decentralized auction allocation
        ├── whca_planner_node.py       # Conflict-Oriented WHCA* (CO-WHCA*) planner
        ├── orca_node.py               # Reciprocal velocity obstacles with dynamic yield
        ├── kinematic_carrier_node.py  # 50 Hz kinematic simulation backend
        ├── safety_supervisor_node.py  # Authoritative hard safety & braking guard
        ├── path_follower_node.py      # Waypoint guidance controller & recovery
        ├── corridor_mutex_node.py     # Ricart-Agrawala narrow aisle mutex
        ├── localization_node.py       # Odometry-to-map frame transformer
        ├── data_collection_node.py    # High-fidelity JSONL telemetry recorder
        └── velocity_profiles.py       # Formal kinematic limits (0.46 m/s default)
```

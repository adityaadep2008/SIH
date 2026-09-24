# Zenoh Middleware Debugging Handoff & Technical Post-Mortem

> **Target Audience**: Incoming AI Debugging Agent / Systems Engineer  
> **Status**: Non-operational for Gazebo Multi-AMR Fleet Simulation (`rmw_zenoh_cpp`)  
> **Workspace**: `/home/rtsws/amr_ws/src/SIH`  
> **Date**: September 24, 2026  

---

## 1. Executive Summary & Mission Context

This repository contains a decentralized multi-robot warehouse fleet software stack designed for 4 TurtleBot 4 AMRs running in Gazebo Sim Harmonic (ROS 2 Jazzy on Ubuntu 24.04). 

* **The Proven Working Baseline**: The fleet runs on **`rmw_cyclonedds_cpp`** with custom XML configuration (`src/sih_amr_fleet/config/cyclonedds.xml`). Under Cyclone DDS, the fleet achieves 100% throughput across 32 consecutive automated work-cycle benchmark runs without a single collision or freeze (verified in `/home/rtsws/amr_ws/log/sih_data_collection/laptop_data_collection_20260924_005415/`).
* **The Goal**: Migrate communication to **Eclipse Zenoh (`rmw_zenoh_cpp`)** to eliminate $O(N^2)$ multicast discovery storms for both simulated and physical WiFi mesh deployments.
* **The Current State**: When running data collection scripts with Zenoh (`run_laptop_data_collection.py`), the simulation either:
  1. **Hangs during robot bringup** (AMR 4 `create_robot` process hangs waiting for `robot_description` topic), or
  2. **Freezes mid-simulation after 30–60 seconds of motion** (AMRs navigate 10–16 meters, then logging halts, RTF drops to 0.15x, and Zenoh reports scope routing and SHM watchdog errors).

---

## 2. System & Middleware Specifications

* **OS**: Ubuntu 24.04 LTS (Noble Numbat) x86_64 (`Linux 7.0.0-31-generic`)
* **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
* **Zenoh Packages**:
  * `ros-jazzy-rmw-zenoh-cpp` (v0.2.x)
  * `ros-jazzy-zenoh-cpp-vendor`
* **Gazebo Simulator**: Gazebo Sim Harmonic 8.11.0 (`gz sim`)
* **GPU / Graphics**: NVIDIA GeForce RTX 3050 Laptop GPU (Driver: 595.84, CUDA: 13.2)
* **Active ROS Environment Variables**:
  * `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  * `ZENOH_SESSION_CONFIG_URI="/home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/config/zenoh_mesh_peer.json5"`
  * `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`

---

## 3. Comprehensive Timeline of Actions & Modifications

### Phase 1: Environment & Discovery Fixes
1. **Discovered Environment Variable Name Discrepancy**:
   * *Original Code*: Sourced `export ZENOH_CONFIG_FILE="..."`.
   * *Forensic Inspection of `librmw_zenoh_cpp.so`*: Revealed that ROS 2 Jazzy's `rmw_zenoh_cpp` reads `ZENOH_SESSION_CONFIG_URI` for ROS nodes and `ZENOH_ROUTER_CONFIG_URI` for the router daemon. `ZENOH_CONFIG_FILE` was completely ignored, causing `rmw_zenoh_cpp` to silently fall back to the default `/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`.
   * *Fix Applied*: Updated `scripts/env_zenoh.sh`, `scripts/setup_zenoh.sh`, and `scripts/launch_fleet_amrs.sh` to export `ZENOH_SESSION_CONFIG_URI`.

2. **Fixed `ROS_AUTOMATIC_DISCOVERY_RANGE`**:
   * *Original Code*: Set `ROS_AUTOMATIC_DISCOVERY_RANGE=ALL`.
   * *Forensic Finding*: ROS 2 Jazzy `rcl` rejected `ALL` with `[WARN] [rcl]: Invalid value 'ALL' specified for 'ROS_AUTOMATIC_DISCOVERY_RANGE', assuming localhost only`. Valid options are `LOCALHOST`, `SUBNET`, `OFF`, `SYSTEM_DEFAULT`.
   * *Fix Applied*: Changed to `LOCALHOST`.

### Phase 2: Configuration Schema Validation
3. **Fixed Schema Crash in `zenoh_mesh_peer.json5`**:
   * *Original Code*:
     ```json5
     "scouting": {
       "multicast": {
         "enabled": true,
         "autoconnect": { "peer": true }
       }
     }
     ```
   * *Forensic Finding*: When `ZENOH_SESSION_CONFIG_URI` was pointed to this file, `rclpy.init()` crashed with:
     ```text
     ERROR zenohc::config: JSON error: Message { msg: "invalid type: boolean `true`, expected a list of whatami variants ('router', 'peer', 'client')" }
     RCLError: failed to initialize rcl: caught C++ exception: Error configuring Zenoh session.
     ```
   * *Fix Applied*: Corrected `autoconnect` to use entity lists:
     ```json5
     "autoconnect": {
       "router": ["router", "peer"],
       "peer": ["router", "peer"]
     }
     ```

4. **Resolved QoS vs `lowlatency` Incompatibility**:
   * *Original Code*: `zenoh_mesh_peer.json5` contained `"lowlatency": true` under unicast transport.
   * *Forensic Finding*: When running `pytest`, 18 ROS 2 nodes failed initialization with:
     ```text
     ERROR zenohc::session: Error opening session: 'qos' and 'lowlatency' options are incompatible.
     ```
   * *Fix Applied*: Removed `lowlatency: true` to allow standard ROS 2 QoS durability. Unit test suite passed 89/89.

### Phase 3: Router Daemon Management & Cleanup
5. **Integrated `rmw_zenohd` into Launch Scripts**:
   * In a 4-AMR simulation, there are over 60 individual ROS 2 nodes across multiple processes. Without a router daemon, inter-process communication between Gazebo bridges and consolidated agents failed.
   * *Fix Applied*: Added automated startup of `ros2 run rmw_zenoh_cpp rmw_zenohd` in `scripts/launch_fleet_amrs.sh` when `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, with `ZENOHD_PID` registered in the `cleanup()` trap.
   * *Fix Applied*: Added `"rmw_zenohd"` and `"zenoh"` to `clean_lingering_processes()` in `scripts/run_laptop_data_collection.py` and `scripts/run_desktop_data_collection.py`.

### Phase 4: Shared Memory Disabling
6. **Disabled Shared Memory in `zenoh_mesh_peer.json5`**:
   * In Run 1, the simulation locked up with:
     ```text
     [warehouse_map_node] WARN zenoh_shm::watchdog::periodic_task: error setting scheduling priority for thread: OS(1), will run with priority 48... SHM subsystem may experience some timeouts in case of an heavy congested system.
     [random_task_generator_node] ERROR zenoh::net::routing::dispatcher::pubsub: Face Route data with unknown scope 8!
     ```
   * *Fix Applied*: Set `"shared_memory": { "enabled": false }` in `src/sih_amr_fleet/config/zenoh_mesh_peer.json5` to route messages exclusively via loopback TCP.

---

## 4. Forensic Breakdown of the Two Reproducible Failure Modes

### Failure Mode A: The Mid-Run Freeze (Run `laptop_data_collection_20260924_193704`)
* **Command Executed**: `python3 scripts/run_laptop_data_collection.py -r 2 -t 20 -f 4 -s 0.46`
* **Log Location**: `/home/rtsws/amr_ws/log/sih_data_collection/laptop_data_collection_20260924_193704/laptop_run_001_20260924_193704/`
* **What Succeeded**:
  * Bringup passed: All 4 AMRs spawned, passed pose verification at charging pads.
  * CBBA Auction passed: 4 tasks announced, all 4 robots submitted bids, reached unanimous commit, and accepted tasks.
  * Motion occurred:
    * `robot_4` navigated **16.4 meters** (from $y=-29.55$ to $y=-13.18$).
    * `robot_3` navigated **10.2 meters** (from $y=-29.55$ to $y=-19.32$).
    * `robot_1` moved **1.0 meter** (from $y=-29.55$ to $y=-28.60$).
* **Where It Failed**:
  * At wall epoch `1790258948` (33s of fleet motion), logging abruptly halted.
  * Telemetry file stopped appending records for over 320 seconds.
  * `kinematic_carrier.log` flooded with:
    ```text
    [Carrier] Gazebo set_pose_vector failed (timeout/unreachable). Sync failures: 2450/6772.
    ```
  * `fleet.log` logged:
    ```text
    ERROR rx-2 ThreadId(08) zenoh::net::routing::dispatcher::pubsub: Face{20, c9f6245c157853f86ad992c91d0a2535} Route data with unknown scope 8!
    ERROR rx-2 ThreadId(08) zenoh::net::routing::dispatcher::pubsub: Face{12, a16319e98a890612045ddbe873ca2da0} Route data with unknown scope 7!
    ```
  * Simulation RTF collapsed to **0.15x**.

### Failure Mode B: The Spawner Hang at AMR 4 (Run `laptop_data_collection_20260924_201357`)
* **Command Executed**: `python3 scripts/run_laptop_data_collection.py -r 2 -t 20 -f 4 -s 0.46`
* **Log Location**: `/home/rtsws/amr_ws/log/sih_data_collection/laptop_data_collection_20260924_201357/laptop_run_001_20260924_201357/`
* **What Succeeded**:
  * `robot_1` spawned and passed interface gate (22.1s).
  * `robot_2` spawned and passed interface gate (36.2s).
  * `robot_3` spawned and passed interface gate (50.0s).
* **Where It Failed**:
  * At 53.0s, launcher initiated `spawn_robot` for `robot_4`.
  * The terminal hung indefinitely at:
    ```text
    [ 53.0s] [SPAWNING AMR] Spawning robot_4 at (-0.75, -29.55, yaw=1.5708...)...
    [ 198.1s] RTF: 0.00x | Tasks:0/20(0%) | R1:IDLE R2:IDLE R3:IDLE R4:IDLE ^C
    ```
  * In `robot_4.log`:
    ```text
    [INFO] [create-7]: process started with pid [25986]
    [create-7] 2026-09-24T14:44:55.001556Z WARN ThreadId(15) zenoh::net::runtime::orchestrator: Scouting delay elapsed before start conditions are met.
    [static_transform_publisher-5] 2026-09-24T14:45:14.846283Z WARN net-0 ThreadId(02) zenoh::net::runtime::orchestrator: Unable to connect to any locator of scouted peer 62b2d0ef26e3c00ae3627c0eaa45c3e5: [udp/192.168.1.6:52023, tcp/192.168.1.6:41247]
    ```
  * Process `create-7` (`ros_gz_sim create -name robot_4 ...`) never received the `robot_description` topic from `robot_state_publisher`, so `robot_4` was never created in Gazebo.

---

## 5. Root Cause Analysis for Incoming Debugger

### Root Cause 1: Hybrid Mode Topology Conflict (Router + P2P Mesh Collision)
Inspect `src/sih_amr_fleet/config/zenoh_mesh_peer.json5`:
```json5
{
  "mode": "peer",
  "scouting": {
    "multicast": { "enabled": true, "autoconnect": { "router": ["router", "peer"], "peer": ["router", "peer"] } },
    "gossip": { "enabled": true, "autoconnect": { "router": ["router", "peer"], "peer": ["router", "peer"] } }
  },
  "connect": {
    "endpoints": ["tcp/127.0.0.1:7447"]
  },
  "listen": {
    "endpoints": ["tcp/0.0.0.0:0", "udp/0.0.0.0:0"]
  }
}
```
**Why this breaks**:
1. All nodes connect to the central router `tcp/127.0.0.1:7447`.
2. Simultaneously, every node listens on dynamic wildcard ports (`tcp/0.0.0.0:0`, `udp/0.0.0.0:0`) and broadcasts UDP multicast discovery packets across the machine's active network interface (`192.168.1.6`).
3. Every node then tries to establish direct unicast peer-to-peer TCP/UDP links to all other nodes over the local WiFi network (`udp/192.168.1.6:52023`).
4. In ROS 2 Jazzy's official default configuration (`DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`), **multicast scouting is explicitly disabled (`enabled: false`)** because all nodes communicate through the router daemon. Enabling multicast scouting with autoconnect to peers spawns an $O(N^2)$ socket storm that overwhelms Zenoh's Tokio runtime, blocking topic delivery for `create_robot`.

### Root Cause 2: `ros_gz_sim create` QoS Latching Issue with `rmw_zenoh_cpp`
* `ros_gz_sim create` is a C++ executable that launches, waits for a single message on `robot_description` (published with `TRANSIENT_LOCAL` durability by `robot_state_publisher`), and terminates.
* In ROS 2 Jazzy `rmw_zenoh_cpp`, late-joining subscribers to `TRANSIENT_LOCAL` topics can fail to retrieve the history cache if the publisher session did not declare a matching durability queryable with the router.
* When `create_robot` misses the latched message, it hangs indefinitely.

---

## 6. Files Modified in Working Tree / Commit

| File Path | Description of Changes |
|---|---|
| `src/sih_amr_fleet/config/zenoh_mesh_peer.json5` | Corrected JSON5 schema, disabled SHM, added router connect endpoint. |
| `scripts/env_zenoh.sh` | Exported `ZENOH_SESSION_CONFIG_URI`, set discovery range to `LOCALHOST`. |
| `scripts/setup_zenoh.sh` | Aligned activation template with `ZENOH_SESSION_CONFIG_URI`. |
| `scripts/launch_fleet_amrs.sh` | Added `rmw_zenohd` background daemon management and cleanup. |
| `scripts/run_laptop_data_collection.py` | Added `rmw_zenohd` to `clean_lingering_processes()`. |
| `scripts/run_desktop_data_collection.py` | Added `rmw_zenohd` to `clean_lingering_processes()`. |
| `src/sih_amr_fleet/test/test_algorithms.py` | Updated assertions in `test_zenoh_config_is_valid_peer_mesh_mode`. |

---

## 7. Recommended Next Steps for the Incoming Agent

1. **Test Pure Router Configuration (Disable Multicast & Ephemeral Listeners)**:
   Update `src/sih_amr_fleet/config/zenoh_mesh_peer.json5` to mirror the official ROS 2 Jazzy session config:
   ```json5
   {
     "mode": "peer",
     "transport": {
       "unicast": { "max_links": 64 },
       "shared_memory": { "enabled": false }
     },
     "scouting": {
       "multicast": { "enabled": false },
       "gossip": {
         "enabled": true,
         "target": { "peer": ["router"] },
         "autoconnect": { "peer": ["router"] }
       }
     },
     "connect": {
       "endpoints": ["tcp/127.0.0.1:7447"]
     },
     "listen": {
       "endpoints": []
     }
   }
   ```
   *Rationale*: Setting `listen.endpoints: []` and disabling multicast scouting ensures nodes communicate purely through `rmw_zenohd` on localhost, preventing socket storms and unreachable WiFi peer errors.

2. **Verify `robot_description` Retrieval with Zenoh**:
   Write a discrete unit test launching `robot_state_publisher` and verifying that a late-joining node can reliably echo `/robot_1/robot_description` under `rmw_zenoh_cpp`.

3. **Fallback Switch (If Fast Baseline Needed)**:
   If the human operator requires immediate dataset collection, revert `scripts/launch_fleet_amrs.sh` line 71 back to:
   ```bash
   export RMW_IMPLEMENTATION="${SIH_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
   export CYCLONEDDS_URI="${SIH_CYCLONEDDS_URI:-file://$SIH_ROOT/src/sih_amr_fleet/config/cyclonedds.xml}"
   ```

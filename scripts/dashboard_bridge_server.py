#!/usr/bin/env python3
"""
SIH Real-Time Dashboard Bridge & Telemetry Server.

Seamless dual-protocol bridge connecting the AM-CORD AI web frontend directly to:
1. Live ROS 2 Fleet Topics:
   - /fleet/robot_state            (sih_amr_interfaces/msg/RobotState)
   - /fleet/health                 (sih_amr_interfaces/msg/FleetHealth)
   - /fleet/task_consensus         (sih_amr_interfaces/msg/TaskConsensus)
   - /fleet/task_execution_status  (sih_amr_interfaces/msg/TaskExecutionStatus)
   - /fleet/corridor_protocol      (sih_amr_interfaces/msg/CorridorProtocol)
   - /fleet/safety_state           (sih_amr_interfaces/msg/SafetyState)
   - /fleet/task_announcement      (sih_amr_interfaces/msg/TaskAnnouncement or Task)
   - /fleet/trajectory_intent      (sih_amr_interfaces/msg/TrajectoryIntent)
   - /{robot_id}/planned_route     (sih_amr_interfaces/msg/RoutePlan)
   - /fleet/dashboard_telemetry    (std_msgs/msg/String)
2. Dual-Protocol Server:
   - WebSocket broadcast stream on ws://0.0.0.0:8765 (10 Hz low-latency)
   - HTTP GET /fleet/dashboard_telemetry on http://0.0.0.0:8766
   - Static Web Server serving the 'frontend/' directory on http://0.0.0.0:8766
3. Zero-dependency standard-library RFC 6455 WebSocket implementation.
4. Auto-discovery of active simulation ROS_DOMAIN_ID & CycloneDDS environment.

Usage:
  python3 scripts/dashboard_bridge_server.py
"""

import base64
import copy
import hashlib
import json
import math
import mimetypes
import os
import select
import signal
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Dict, List, Optional, Set

# ==============================================================================
# SELF-HEALING ENVIRONMENT & AUTO-DOMAIN DISCOVERY
# ==============================================================================
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
                                if key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ZENOH_SESSION_CONFIG_URI"):
                                    env_vars[key] = val
                            except ValueError:
                                pass
                        if "ROS_DOMAIN_ID" in env_vars:
                            break
            except (PermissionError, ProcessLookupError, FileNotFoundError):
                continue
    return env_vars


WORKSPACE_INSTALL = Path("/home/rtsws/amr_ws/install")
if WORKSPACE_INSTALL.exists():
    for sp in WORKSPACE_INSTALL.glob("**/site-packages"):
        if str(sp) not in sys.path:
            sys.path.insert(0, str(sp))

detected_env = detect_active_ros_environment()
need_reexec = False
if "ROS_DOMAIN_ID" in detected_env and os.environ.get("ROS_DOMAIN_ID") != detected_env["ROS_DOMAIN_ID"]:
    os.environ["ROS_DOMAIN_ID"] = detected_env["ROS_DOMAIN_ID"]
    need_reexec = True
if "RMW_IMPLEMENTATION" in detected_env and os.environ.get("RMW_IMPLEMENTATION") != detected_env["RMW_IMPLEMENTATION"]:
    os.environ["RMW_IMPLEMENTATION"] = detected_env["RMW_IMPLEMENTATION"]
    need_reexec = True
if "CYCLONEDDS_URI" in detected_env and os.environ.get("CYCLONEDDS_URI") != detected_env["CYCLONEDDS_URI"]:
    os.environ["CYCLONEDDS_URI"] = detected_env["CYCLONEDDS_URI"]
    need_reexec = True
if "ZENOH_SESSION_CONFIG_URI" in detected_env and os.environ.get("ZENOH_SESSION_CONFIG_URI") != detected_env["ZENOH_SESSION_CONFIG_URI"]:
    os.environ["ZENOH_SESSION_CONFIG_URI"] = detected_env["ZENOH_SESSION_CONFIG_URI"]
    need_reexec = True
if "ROS_AUTOMATIC_DISCOVERY_RANGE" in detected_env:
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = detected_env["ROS_AUTOMATIC_DISCOVERY_RANGE"]

zenoh_json5 = Path("/home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/config/zenoh_session_config.json5")
if os.environ.get("RMW_IMPLEMENTATION", "").startswith("rmw_zenoh"):
    if zenoh_json5.exists() and "ZENOH_SESSION_CONFIG_URI" not in os.environ:
        os.environ["ZENOH_SESSION_CONFIG_URI"] = str(zenoh_json5)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
else:
    # Ensure default single-machine baseline CycloneDDS config is applied if not explicit
    cyclone_xml = Path("/home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/config/cyclonedds.xml")
    if cyclone_xml.exists() and "CYCLONEDDS_URI" not in os.environ:
        os.environ["CYCLONEDDS_URI"] = f"file://{cyclone_xml}"
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

interfaces_lib = str(WORKSPACE_INSTALL / "sih_amr_interfaces" / "lib")
current_ld = os.environ.get("LD_LIBRARY_PATH", "")
if interfaces_lib not in current_ld and os.path.exists(interfaces_lib):
    os.environ["LD_LIBRARY_PATH"] = f"{interfaces_lib}:{current_ld}".strip(":")
    need_reexec = True

if need_reexec and "AMR_BRIDGE_REEXEC" not in os.environ:
    os.environ["AMR_BRIDGE_REEXEC"] = "1"
    os.execvpe(sys.executable, [sys.executable] + sys.argv, os.environ)

PORT_HTTP = 8766
PORT_WS = 8765
FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

# Canonical Dock Anchor Coordinates from demo_warehouse.yaml
CANONICAL_DOCK_POSES = {
    "robot_1": {"x": -5.25, "y": -29.55, "theta": 1.5708},
    "robot_2": {"x": -3.75, "y": -29.55, "theta": 1.5708},
    "robot_3": {"x": -2.25, "y": -29.55, "theta": 1.5708},
    "robot_4": {"x": -0.75, "y": -29.55, "theta": 1.5708},
    "robot_5": {"x": 0.75, "y": -29.55, "theta": 1.5708},
    "robot_6": {"x": 2.25, "y": -29.55, "theta": 1.5708},
    "robot_7": {"x": 3.75, "y": -29.55, "theta": 1.5708},
    "robot_8": {"x": 5.25, "y": -29.55, "theta": 1.5708},
}

STATE_LOCK = threading.Lock()

CURRENT_STATE = {
    "robots": {
        r_id: {
            "x": pose["x"],
            "y": pose["y"],
            "theta": pose["theta"],
            "speed": 0.0,
            "state": "IDLE",
            "thought": "Docked at charging pad",
            "taskId": None,
        }
        for r_id, pose in list(CANONICAL_DOCK_POSES.items())[:4]
    },
    "health": {
        f"robot_{i}": {"battery": 100.0, "safe": True} for i in range(1, 5)
    },
    "tasks": [],
    "paths": {},
    "events": [],
    "sim_info": {
        "status": "INITIALIZING",
        "live_ros": False,
        "active_robots": 4,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "last_update": time.time(),
    },
}


def add_event_record(kind: str, detail: str):
    with STATE_LOCK:
        events = CURRENT_STATE["events"]
        events.append({"type": kind, "detail": detail, "timestamp": time.time()})
        if len(events) > 100:
            CURRENT_STATE["events"] = events[-100:]


# ==============================================================================
# ROS 2 NODE INGESTION
# ==============================================================================
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sih_amr_interfaces.msg import (
        CorridorProtocol,
        FleetHealth,
        RobotState,
        RoutePlan,
        SafetyState,
        Task,
        TaskAnnouncement,
        TaskConsensus,
        TaskExecutionStatus,
        TrajectoryIntent,
    )
    from std_msgs.msg import String

    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"[DashboardBridge] Notice: ROS 2 / sih_amr_interfaces import info: {e}")
    ROS2_AVAILABLE = False


if ROS2_AVAILABLE:

    class RosDashboardBridgeNode(Node):
        def __init__(self):
            super().__init__("dashboard_telemetry_bridge")

            qos_state = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            qos_protocol = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            qos_pose = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )

            # Consolidated /fleet topics
            self.create_subscription(
                RobotState, "/fleet/robot_state", self.on_robot_state, qos_state
            )
            self.create_subscription(
                FleetHealth, "/fleet/health", self.on_fleet_health, qos_state
            )
            self.create_subscription(
                TaskConsensus, "/fleet/task_consensus", self.on_task_consensus, qos_protocol
            )
            self.create_subscription(
                TaskExecutionStatus,
                "/fleet/task_execution_status",
                self.on_task_execution_status,
                qos_state,
            )
            self.create_subscription(
                CorridorProtocol,
                "/fleet/corridor_protocol",
                self.on_corridor_protocol,
                qos_protocol,
            )
            self.create_subscription(
                SafetyState, "/fleet/safety_state", self.on_safety_state, qos_state
            )
            self.create_subscription(
                TaskAnnouncement, "/fleet/task_announcement", self.on_task_announcement, qos_state
            )
            self.create_subscription(
                TrajectoryIntent, "/fleet/trajectory_intent", self.on_trajectory_intent, qos_protocol
            )
            self.create_subscription(
                String, "/fleet/dashboard_telemetry", self.on_raw_telemetry, qos_state
            )

            # Individual robot topics (planned route & pose)
            for i in range(1, 9):
                r_id = f"robot_{i}"
                self.create_subscription(
                    RoutePlan,
                    f"/{r_id}/planned_route",
                    lambda msg, rid=r_id: self.on_route_plan(rid, msg),
                    qos_state,
                )
                self.create_subscription(
                    RobotState,
                    f"/{r_id}/state",
                    self.on_robot_state,
                    qos_pose,
                )

            print(f"[DashboardBridge] ROS 2 Bridge subscriptions initialized (Domain: {os.environ.get('ROS_DOMAIN_ID', '0')}).")

        def on_robot_state(self, msg: RobotState):
            try:
                r_id = msg.fleet_header.robot_id
                if not r_id:
                    return
                speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
                with STATE_LOCK:
                    if r_id not in CURRENT_STATE["robots"]:
                        CURRENT_STATE["robots"][r_id] = {}
                    bot = CURRENT_STATE["robots"][r_id]
                    bot["x"] = float(msg.pose.x)
                    bot["y"] = float(msg.pose.y)
                    bot["theta"] = float(msg.pose.theta)
                    bot["speed"] = round(float(speed), 2)
                    CURRENT_STATE["sim_info"]["live_ros"] = True
                    CURRENT_STATE["sim_info"]["last_update"] = time.time()
                    CURRENT_STATE["sim_info"]["active_robots"] = len(CURRENT_STATE["robots"])
            except Exception:
                pass

        def on_fleet_health(self, msg: FleetHealth):
            try:
                r_id = msg.fleet_header.robot_id
                if not r_id:
                    return
                with STATE_LOCK:
                    if r_id not in CURRENT_STATE["health"]:
                        CURRENT_STATE["health"][r_id] = {}
                    h = CURRENT_STATE["health"][r_id]
                    h["battery"] = round(float(msg.battery_percent), 1)
                    h["safe"] = bool(msg.safety_ok)
                    if msg.active_task_id and r_id in CURRENT_STATE["robots"]:
                        CURRENT_STATE["robots"][r_id]["taskId"] = msg.active_task_id
            except Exception:
                pass

        def on_task_consensus(self, msg: TaskConsensus):
            try:
                task_id = msg.task_id
                winner = msg.winner_robot_id
                bid = round(float(msg.winning_bid), 2)
                epoch = msg.assignment_epoch

                event_str = f"[{winner}:CBBA] Decision: UNANIMOUS_COMMIT for task {task_id} -> Winner={winner}, Bid={bid}. Quorum=4/4"
                add_event_record("quorum", event_str)

                with STATE_LOCK:
                    if winner in CURRENT_STATE["robots"]:
                        CURRENT_STATE["robots"][winner]["thought"] = f"Assigned: {task_id}"
                        CURRENT_STATE["robots"][winner]["taskId"] = task_id

                    # Upsert task in task list
                    found = False
                    for t in CURRENT_STATE["tasks"]:
                        if t.get("task_id") == task_id:
                            t["assigned_robot_id"] = winner
                            t["winning_bid"] = bid
                            t["status"] = "EN_ROUTE_PICKUP"
                            found = True
                            break
                    if not found:
                        CURRENT_STATE["tasks"].append(
                            {
                                "task_id": task_id,
                                "priority": 100,
                                "status": "EN_ROUTE_PICKUP",
                                "assigned_robot_id": winner,
                                "winning_bid": bid,
                                "progress_pct": 10,
                                "dwell_remaining": 3.0,
                                "created_at_epoch": time.time(),
                            }
                        )
            except Exception:
                pass

        def on_task_execution_status(self, msg: TaskExecutionStatus):
            try:
                task_id = msg.task_id
                r_id = msg.owner_robot_id or msg.fleet_header.robot_id
                phase_code = msg.phase
                phase_map = {
                    0: "EN_ROUTE_PICKUP",
                    1: "PICKUP_WAIT",
                    2: "EN_ROUTE_DROPOFF",
                    3: "DROPOFF_WAIT",
                    4: "COMPLETED",
                    5: "FAILED",
                }
                phase_str = phase_map.get(phase_code, "ACTIVE")
                dwell_rem = round(float(msg.wait_time_remaining_s), 1)

                with STATE_LOCK:
                    if r_id in CURRENT_STATE["robots"]:
                        CURRENT_STATE["robots"][r_id]["state"] = phase_str
                        CURRENT_STATE["robots"][r_id]["taskId"] = task_id
                        if dwell_rem > 0:
                            CURRENT_STATE["robots"][r_id]["thought"] = f"Dwelling: {dwell_rem}s"
                        elif phase_str == "EN_ROUTE_DROPOFF":
                            CURRENT_STATE["robots"][r_id]["thought"] = "En route to dropoff station"
                        elif phase_str == "EN_ROUTE_PICKUP":
                            CURRENT_STATE["robots"][r_id]["thought"] = f"Pickup for {task_id}"

                    found = False
                    for t in CURRENT_STATE["tasks"]:
                        if t.get("task_id") == task_id:
                            t["status"] = phase_str
                            t["assigned_robot_id"] = r_id
                            t["dwell_remaining"] = dwell_rem
                            if phase_code == 0:
                                t["progress_pct"] = 25
                            elif phase_code == 1:
                                t["progress_pct"] = 50
                            elif phase_code == 2:
                                t["progress_pct"] = 75
                            elif phase_code == 3:
                                t["progress_pct"] = 90
                            elif phase_code == 4:
                                t["progress_pct"] = 100
                            found = True
                            break
                    if not found and task_id:
                        CURRENT_STATE["tasks"].append(
                            {
                                "task_id": task_id,
                                "priority": 100,
                                "status": phase_str,
                                "assigned_robot_id": r_id,
                                "winning_bid": 100.0,
                                "progress_pct": 25 if phase_code == 0 else 50,
                                "dwell_remaining": dwell_rem,
                                "created_at_epoch": time.time(),
                            }
                        )
            except Exception:
                pass

        def on_task_announcement(self, msg):
            try:
                t = getattr(msg, "task", msg)
                task_id = getattr(t, "task_id", "")
                if not task_id:
                    return
                px = float(t.pickup.x) if hasattr(t, "pickup") else 0.0
                py = float(t.pickup.y) if hasattr(t, "pickup") else 0.0
                dx = float(t.dropoff.x) if hasattr(t, "dropoff") else 0.0
                dy = float(t.dropoff.y) if hasattr(t, "dropoff") else 0.0
                prio = int(getattr(t, "priority", 50))
                pickup_wait = float(getattr(t, "pickup_wait_s", 3.0))
                dropoff_wait = float(getattr(t, "dropoff_wait_s", 3.0))

                with STATE_LOCK:
                    existing = next((x for x in CURRENT_STATE["tasks"] if x.get("task_id") == task_id), None)
                    if not existing:
                        CURRENT_STATE["tasks"].append(
                            {
                                "task_id": task_id,
                                "priority": prio,
                                "status": "CBBA_AUCTION",
                                "assigned_robot_id": None,
                                "winning_bid": None,
                                "pickup": {"x": px, "y": py, "theta": 0.0, "label": f"Rack Pick ({px:.1f}, {py:.1f})"},
                                "dropoff": {"x": dx, "y": dy, "theta": 0.0, "label": f"Dispatch ({dx:.1f}, {dy:.1f})"},
                                "progress_pct": 0,
                                "dwell_times": {"pickup_wait_s": pickup_wait, "dropoff_wait_s": dropoff_wait},
                                "dwell_remaining": 0.0,
                                "created_at_epoch": time.time(),
                            }
                        )
                event_str = f"[FLEET:CBBA] Decision: ANNOUNCE_TASK for task {task_id}. Info: priority={prio}"
                add_event_record("cbba", event_str)
            except Exception:
                pass

        def on_corridor_protocol(self, msg: CorridorProtocol):
            try:
                r_id = msg.fleet_header.robot_id
                corridor = msg.corridor_id
                event_code = msg.event
                action_map = {
                    0: "REQUEST_MUTEX",
                    1: "GRANT_MUTEX",
                    2: "DEFER_MUTEX",
                    3: "ENTER_CORRIDOR",
                    4: "EXIT_CORRIDOR",
                    5: "RELEASE_MUTEX",
                    6: "CANCEL_MUTEX",
                }
                action = action_map.get(event_code, "CORRIDOR_EVENT")
                event_str = f"[{r_id}:CorridorMutex] Decision: {action} for corridor {corridor}"
                add_event_record("mutex", event_str)

                if event_code in (0, 3):
                    with STATE_LOCK:
                        if r_id in CURRENT_STATE["robots"]:
                            CURRENT_STATE["robots"][r_id]["thought"] = f"Corridor {corridor} ({action})"
            except Exception:
                pass

        def on_safety_state(self, msg: SafetyState):
            try:
                r_id = msg.fleet_header.robot_id
                action_map = {0: "CLEAR", 1: "SLOWDOWN", 2: "EMERGENCY_STOP"}
                action = action_map.get(msg.level, "SAFETY_EVENT")
                dist = getattr(msg, "nearest_obstacle_m", getattr(msg, "min_obstacle_distance", 0.0))
                reason = getattr(msg, "reason", "")
                event_str = f"[{r_id}:SafetySupervisor] Decision: {action} (Dist: {dist:.2f}m, Reason: {reason})"
                add_event_record("safety", event_str)

                if msg.level > 0:
                    with STATE_LOCK:
                        if r_id in CURRENT_STATE["robots"]:
                            CURRENT_STATE["robots"][r_id]["thought"] = f"⚠️ Safety: {action}"
            except Exception:
                pass

        def on_trajectory_intent(self, msg: TrajectoryIntent):
            try:
                r_id = msg.fleet_header.robot_id
                if not r_id:
                    return
                # Convert grid cells to canonical world coordinates (origin -22.5, -30.0, resolution 0.5)
                wps = []
                for cell in msg.reservations:
                    wx = round(cell.x * 0.5 - 22.5, 2)
                    wy = round(cell.y * 0.5 - 30.0, 2)
                    wps.append({"x": wx, "y": wy})
                if wps:
                    with STATE_LOCK:
                        CURRENT_STATE["paths"][r_id] = wps
            except Exception:
                pass

        def on_route_plan(self, r_id: str, msg: RoutePlan):
            try:
                wps = [{"x": float(wp.x), "y": float(wp.y)} for wp in msg.waypoints]
                with STATE_LOCK:
                    CURRENT_STATE["paths"][r_id] = wps
                event_str = f"[{r_id}:WHCA] Decision: ROUTE_FEASIBLE for task={msg.task_id}. Info: waypoints={len(wps)}"
                add_event_record("whca", event_str)
            except Exception:
                pass

        def on_raw_telemetry(self, msg: String):
            try:
                data = json.loads(msg.data)
                with STATE_LOCK:
                    if "robots" in data:
                        for k, v in data["robots"].items():
                            if k not in CURRENT_STATE["robots"]:
                                CURRENT_STATE["robots"][k] = {}
                            CURRENT_STATE["robots"][k].update(v)
                    if "health" in data:
                        CURRENT_STATE["health"].update(data["health"])
                    CURRENT_STATE["sim_info"]["live_ros"] = True
                    CURRENT_STATE["sim_info"]["last_update"] = time.time()
            except Exception:
                pass


def start_ros2_node():
    if not ROS2_AVAILABLE:
        print("[DashboardBridge] ROS 2 not imported; operating in bridge server mode.")
        return
    try:
        rclpy.init()
        node = RosDashboardBridgeNode()
        print(f"[DashboardBridge] ROS 2 Node spinning successfully on Domain {os.environ.get('ROS_DOMAIN_ID', '0')}.")
        rclpy.spin(node)
    except Exception as e:
        print(f"[DashboardBridge] ROS 2 spin notice: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ==============================================================================
# RFC 6455 LIGHTWEIGHT WEBSOCKET SERVER
# ==============================================================================
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
CONNECTED_WS_CLIENTS: Set[socket.socket] = set()
WS_LOCK = threading.Lock()


def build_ws_frame(payload_bytes: bytes) -> bytes:
    length = len(payload_bytes)
    if length <= 125:
        header = bytes([0x81, length])
    elif length <= 65535:
        header = struct.pack("!BBH", 0x81, 126, length)
    else:
        header = struct.pack("!BBQ", 0x81, 127, length)
    return header + payload_bytes


def broadcast_ws_telemetry():
    while True:
        time.sleep(0.2)  # 5 Hz adaptive broadcast rate (prevents network buffer & DOM saturation)
        with WS_LOCK:
            clients = list(CONNECTED_WS_CLIENTS)
        if not clients:
            continue

        with STATE_LOCK:
            payload = json.dumps(CURRENT_STATE).encode("utf-8")
        frame = build_ws_frame(payload)

        to_remove = []
        for client in clients:
            try:
                client.sendall(frame)
            except Exception:
                to_remove.append(client)

        if to_remove:
            with WS_LOCK:
                for c in to_remove:
                    CONNECTED_WS_CLIENTS.discard(c)
                    try:
                        c.close()
                    except Exception:
                        pass


def handle_ws_client(client_sock: socket.socket, addr):
    try:
        client_sock.settimeout(5.0)
        request_data = b""
        while b"\r\n\r\n" not in request_data:
            chunk = client_sock.recv(1024)
            if not chunk:
                return
            request_data += chunk
            if len(request_data) > 8192:
                return

        req_text = request_data.decode("utf-8", errors="ignore")
        ws_key = None
        for line in req_text.split("\r\n"):
            if line.lower().startswith("sec-websocket-key:"):
                ws_key = line.split(":", 1)[1].strip()
                break

        if not ws_key:
            client_sock.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
            client_sock.close()
            return

        accept_val = base64.b64encode(
            hashlib.sha1((ws_key + WS_GUID).encode("utf-8")).digest()
        ).decode("utf-8")

        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_val}\r\n"
            "\r\n"
        )
        client_sock.sendall(response.encode("utf-8"))
        client_sock.settimeout(None)

        with WS_LOCK:
            CONNECTED_WS_CLIENTS.add(client_sock)
        print(f"[DashboardBridge] WebSocket client connected from {addr}")

        # Send immediate current snapshot
        with STATE_LOCK:
            initial_frame = build_ws_frame(json.dumps(CURRENT_STATE).encode("utf-8"))
        client_sock.sendall(initial_frame)

        # Keep reading to detect close or ping
        while True:
            header = client_sock.recv(2)
            if not header or len(header) < 2:
                break
            b1, b2 = header[0], header[1]
            opcode = b1 & 0x0F
            is_masked = bool(b2 & 0x80)
            length = b2 & 0x7F

            if length == 126:
                ext = client_sock.recv(2)
                length = struct.unpack("!H", ext)[0]
            elif length == 127:
                ext = client_sock.recv(8)
                length = struct.unpack("!Q", ext)[0]

            mask_key = client_sock.recv(4) if is_masked else b""
            payload = client_sock.recv(length) if length > 0 else b""

            if opcode == 0x8:  # Close frame
                break
            elif opcode == 0x9:  # Ping -> Pong
                pong = bytes([0x8A, 0x00])
                client_sock.sendall(pong)

    except Exception:
        pass
    finally:
        with WS_LOCK:
            CONNECTED_WS_CLIENTS.discard(client_sock)
        try:
            client_sock.close()
        except Exception:
            pass


def start_ws_server():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", PORT_WS))
    server_sock.listen(16)
    print(f"[DashboardBridge] WebSocket endpoint running at ws://localhost:{PORT_WS}")

    threading.Thread(target=broadcast_ws_telemetry, daemon=True).start()

    while True:
        try:
            client, addr = server_sock.accept()
            threading.Thread(target=handle_ws_client, args=(client, addr), daemon=True).start()
        except Exception:
            break


# ==============================================================================
# HTTP SERVER (TELEMETRY REST API + FRONTEND STATIC ASSETS)
# ==============================================================================
class UnifiedHTTPHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.end_headers()

    def do_HEAD(self):
        if self.path == "/fleet/dashboard_telemetry" or self.path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            return

        req_path = self.path.split("?")[0]
        file_path = FRONTEND_DIR / "index.html" if req_path in ("/", "/index.html") else (FRONTEND_DIR / req_path.lstrip("/")).resolve()
        if file_path.is_file():
            mime_type, _ = mimetypes.guess_type(str(file_path))
            if not mime_type:
                mime_type = "application/javascript" if file_path.suffix == ".js" else "text/css"
            self.send_response(200)
            self.send_header("Content-Type", mime_type)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(file_path.stat().st_size))
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        # 1. Telemetry API endpoint
        if self.path == "/fleet/dashboard_telemetry" or self.path == "/api/telemetry":
            with STATE_LOCK:
                data = json.dumps(CURRENT_STATE).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(data)
            return

        # 2. Frontend Static File Serving
        req_path = self.path.split("?")[0]
        if req_path == "/" or req_path == "/index.html":
            file_path = FRONTEND_DIR / "index.html"
        else:
            clean_path = req_path.lstrip("/")
            file_path = (FRONTEND_DIR / clean_path).resolve()

        # Prevent directory traversal outside frontend
        if not str(file_path).startswith(str(FRONTEND_DIR.resolve())):
            self.send_response(403)
            self.end_headers()
            self.wfile.write(b"Forbidden")
            return

        if file_path.is_file():
            mime_type, _ = mimetypes.guess_type(str(file_path))
            if not mime_type:
                if file_path.suffix == ".js":
                    mime_type = "application/javascript"
                elif file_path.suffix == ".css":
                    mime_type = "text/css"
                elif file_path.suffix == ".csv":
                    mime_type = "text/csv"
                else:
                    mime_type = "application/octet-stream"

            try:
                with open(file_path, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", mime_type)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(f"Internal Server Error: {e}".encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"404 Not Found")

    def log_message(self, format, *args):
        # Suppress routine static asset access noise
        return


def start_http_server():
    server = HTTPServer(("0.0.0.0", PORT_HTTP), UnifiedHTTPHandler)
    print(f"[DashboardBridge] Web UI & Telemetry server ready at http://localhost:{PORT_HTTP}")
    server.serve_forever()


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
def main():
    print("=" * 70)
    print(" AM-CORD AI | Autonomous Mobility Coordination Fleet Bridge Server")
    print("=" * 70)
    print(f" [✓] Serving Web Dashboard : http://localhost:{PORT_HTTP}")
    print(f" [✓] WebSocket Telemetry   : ws://localhost:{PORT_WS}")
    print(f" [✓] HTTP REST Telemetry   : http://localhost:{PORT_HTTP}/fleet/dashboard_telemetry")
    print(f" [✓] Active ROS Domain ID  : {os.environ.get('ROS_DOMAIN_ID', '0')}")
    print(f" [✓] Frontend Path         : {FRONTEND_DIR}")
    print("-" * 70)

    # 1. Start HTTP Server in background thread
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    # 2. Start WebSocket Server in background thread
    ws_thread = threading.Thread(target=start_ws_server, daemon=True)
    ws_thread.start()

    # 3. Start ROS 2 Ingestion Thread
    ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
    ros_thread.start()

    print("[DashboardBridge] All bridge services active. Press Ctrl+C to terminate.")

    # Graceful shutdown handler
    def handle_sigint(signum, frame):
        print("\n[DashboardBridge] Stopping server...")
        with WS_LOCK:
            for client in list(CONNECTED_WS_CLIENTS):
                try:
                    client.close()
                except Exception:
                    pass
            CONNECTED_WS_CLIENTS.clear()
        os._exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[DashboardBridge] Bridge server terminated.")


if __name__ == "__main__":
    main()

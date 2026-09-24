#!/usr/bin/env bash
# Start a clean warehouse, then insert four TurtleBot 4 AMRs sequentially.
# Ctrl+C from this terminal stops every process started by this script.

set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIH_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${AMR_WS:-$(cd "$SIH_ROOT/../.." && pwd)}"
POSE_FILE="${SIH_AMR_POSES_FILE:-$HOME/.config/sih_amr_poses.env}"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-$WORKSPACE/src/warehouse_world_custom}"
OVERLAY="${OVERLAY:-$WORKSPACE/install}"
WORLD_FILE="${WORLD_FILE:-$WAREHOUSE_DIR/worlds/small_warehouse/warehouse_clean.sdf}"
WORLD_NAME="${WORLD_NAME:-default}"
MODEL="${MODEL:-lite}"
SENSOR_PROFILE="${SENSOR_PROFILE:-fleet}"
LIDAR_UPDATE_RATE_HZ="${LIDAR_UPDATE_RATE_HZ:-10.0}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
GUI_RENDER_ENGINE="${GUI_RENDER_ENGINE:-ogre2}"
GUI_CONFIG="${GZ_GUI_CONFIG:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config}"
START_GUI="${START_GUI:-true}"
START_CHARGING="${START_CHARGING:-false}"
START_FLEET="${START_FLEET:-false}"
FLEET_RANDOM_TASKS="${FLEET_RANDOM_TASKS:-true}"
CONTROL_CONFIG="${SIH_CONTROL_CONFIG:-$SIH_ROOT/src/sih_amr_fleet/config/fleet_fast_control.yaml}"
export SIH_CONTROL_CONFIG="$CONTROL_CONFIG"
FLEET_RECORD_DATA="${FLEET_RECORD_DATA:-true}"
FLEET_SCENARIO_FILE="${FLEET_SCENARIO_FILE:-}"
# Tracking speed profile; default 0.46 m/s (physical_max test point)
FLEET_TRACKING_SPEED_MPS="${FLEET_TRACKING_SPEED_MPS:-0.46}"
FLEET_RESERVATION_SLOT_S="${FLEET_RESERVATION_SLOT_S:-0.0}"
SPAWN_WAIT_SECONDS="${SPAWN_WAIT_SECONDS:-180}"
SETTLE_SECONDS="${SETTLE_SECONDS:-20}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$WORKSPACE/log/four_amr_runs/$RUN_ID}"
FLEET_DATA_FILE="${FLEET_DATA_FILE:-$LOG_DIR/fleet_telemetry.jsonl}"
RUN_EVENTS_FILE="$LOG_DIR/run_events.jsonl"

[[ -f "$POSE_FILE" ]] || { echo "ERROR: pose file not found: $POSE_FILE" >&2; exit 2; }
source "$POSE_FILE"
for robot in 1 2 3 4; do
  for axis in X Y YAW; do
    variable="ROBOT_${robot}_${axis}"
    [[ -n "${!variable:-}" ]] || { echo "ERROR: set $variable in $POSE_FILE" >&2; exit 2; }
  done
done
[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 2; }
[[ -f "$GUI_CONFIG" ]] || { echo "ERROR: GUI config not found: $GUI_CONFIG" >&2; exit 2; }
mkdir -p "$LOG_DIR" || { echo "ERROR: cannot create $LOG_DIR" >&2; exit 1; }
write_run_event() {
  printf '{"event_type":"%s","wall_epoch_s":%s,"detail":"%s"}\n' "$1" "$(date +%s)" "$2" >> "$RUN_EVENTS_FILE"
}
write_run_event launcher_started "headless_or_gui_run_requested"
write_run_event simulation_profile "sensor_profile=$SENSOR_PROFILE lidar_hz=$LIDAR_UPDATE_RATE_HZ tracking_mps=$FLEET_TRACKING_SPEED_MPS reservation_slot_s=$FLEET_RESERVATION_SLOT_S"

source /opt/ros/jazzy/setup.bash
source "$OVERLAY/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# The 60-process graph repeatedly left valid Fast DDS endpoints unmatched even
# after shared memory was disabled.  Cyclone DDS is installed with Jazzy on
# this host and is now the deterministic single-machine baseline.  Do not
# inherit an unrelated shell-wide RMW selection; comparison runs opt in with
# the project-specific SIH_RMW_IMPLEMENTATION variable.
export RMW_IMPLEMENTATION="${SIH_RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_zenoh* ]]; then
  export ZENOH_CONFIG_FILE="${SIH_ZENOH_CONFIG:-$SIH_ROOT/src/sih_amr_fleet/config/zenoh_mesh_peer.json5}"
  export ZENOH_ROUTER_MODE=peer
elif [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps* ]]; then
  export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-SYNCHRONOUS}"
  if [[ -n "${SIH_FASTDDS_PROFILE:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SIH_FASTDDS_PROFILE"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  fi
fi
write_run_event middleware_selected "$RMW_IMPLEMENTATION"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM=xcb
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIH_ROOT/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

SERVER_PID="" CLOCK_PID="" GUI_PID="" FLEET_PID="" DATA_PID="" STARTED_PID=""
declare -a ROBOT_PIDS=() CHARGING_PIDS=()

start_group() {
  local log_file="$1"
  shift
  setsid nohup "$@" >"$log_file" 2>&1 < /dev/null &
  STARTED_PID=$!
}

cleanup() {
  local status=$? pid
  trap - EXIT INT TERM
  write_run_event launcher_exiting "status=$status"
  echo 'Stopping this four-AMR run...'
  for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
    # start_group creates a dedicated session.  Signal both its leader and
    # its process group: `timeout` can otherwise interrupt this wrapper while
    # a ROS launch has already re-parented its children, leaving a stale AMR
    # group that blocks the next clean headless run.
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 15); do
    local alive=false
    for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
      [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null && alive=true
    done
    [[ "$alive" == false ]] && break
    sleep 1
  done
  for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
    [[ -n "$pid" ]] && kill -KILL "$pid" 2>/dev/null || true
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null || true
  done
  if [[ -f "$FLEET_DATA_FILE" ]]; then
    python3 -c "
import json
first_s, first_w = None, None
last_s, last_w = None, None
try:
    with open('$FLEET_DATA_FILE') as f:
        for line in f:
            if not line.strip(): continue
            r = json.loads(line)
            st = float(r.get('logged_at', r.get('sim_time_s', 0.0)))
            wt = float(r.get('wall_logged_at', r.get('wall_time_s', 0.0)))
            if st <= 0.0 or wt <= 0.0: continue
            if first_s is None: first_s, first_w = st, wt
            last_s, last_w = st, wt
    if first_s is not None and last_s is not None and last_w > first_w:
        d_sim = last_s - first_s
        d_wall = last_w - first_w
        rtf = d_sim / d_wall
        print(f'\n=== Real-Time Ratio (RTF): {rtf:.2f}x (Simulated: {d_sim:.1f}s, Wall: {d_wall:.1f}s) ===\n')
except Exception:
    pass
" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for() {
  local timeout="$2"
  shift 2
  for _ in $(seq 1 "$timeout"); do "$@" && return 0; sleep 1; done
  return 1
}
model_exists() {
  local robot="$1" log_file="$LOG_DIR/$1.log"
  if [[ -f "$log_file" ]] && grep -Fq "[$robot.create_robot]: Entity creation successful" "$log_file" 2>/dev/null; then
    return 0
  fi
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq "^[[:space:]]*-[[:space:]]+$robot/turtlebot4[[:space:]]*$"
}
warehouse_ready() {
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq '^[[:space:]]*-[[:space:]]+charging_pad_1[[:space:]]*$'
}
fail() { echo "ERROR: $*" >&2; echo "Logs: $LOG_DIR" >&2; exit 1; }

# Clean up and ensure no lingering processes from prior crashed/aborted runs can hijack simulation
if pgrep -f '[g]z sim|[r]os_gz_bridge|[s]pawn_minimal_amr|[t]urtlebot4_spawn|[k]inematic_carrier' >/dev/null; then
  echo "Detected lingering simulator or carrier processes; cleaning up before run..."
  pkill -15 -f '[g]z sim|[r]os_gz_bridge|[s]pawn_minimal_amr|[t]urtlebot4_spawn|[k]inematic_carrier' 2>/dev/null || true
  sleep 1.0
  pkill -9 -f '[g]z sim|[r]os_gz_bridge|[s]pawn_minimal_amr|[t]urtlebot4_spawn|[k]inematic_carrier' 2>/dev/null || true
  sleep 0.5
fi
if pgrep -f '[g]z sim|[r]os_gz_bridge|[s]pawn_minimal_amr|[t]urtlebot4_spawn|[k]inematic_carrier' >/dev/null; then
  fail 'A Gazebo, carrier, or AMR launch is still active and could not be terminated. Stop it before starting a clean run.'
fi

echo "Validating physics timing invariant (controller_update_rate <= 1 / max_step_size)..."
python3 -m sih_amr_fleet.physics_validator --world "$WORLD_FILE" --control-config "$CONTROL_CONFIG" || fail 'Physics and controller timing invariant failed'

echo "Run logs: $LOG_DIR"
echo "Starting server with $RENDER_ENGINE..."
start_group "$LOG_DIR/gazebo_server.log" gz sim -s -r --render-engine "$RENDER_ENGINE" "$WORLD_FILE"
SERVER_PID="$STARTED_PID"
write_run_event gazebo_server_started "pid=$SERVER_PID"
wait_for server 60 kill -0 "$SERVER_PID" || fail 'Gazebo server exited during startup'
wait_for warehouse 60 warehouse_ready || fail 'Gazebo did not expose the warehouse models'

start_group "$LOG_DIR/clock_bridge.log" ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
CLOCK_PID="$STARTED_PID"
sleep 3
kill -0 "$CLOCK_PID" 2>/dev/null || fail 'Clock bridge exited'

# Start telemetry before any robot/controller process so /rosout diagnostics
# from bring-up are captured too. Direct fleet.launch.py use still supports its
# own record_data node; this launcher disables that duplicate below.
if [[ "$START_FLEET" == true && "$FLEET_RECORD_DATA" == true ]]; then
  start_group "$LOG_DIR/data_collection.log" ros2 run sih_amr_fleet data_collection_node \
    --ros-args -p use_sim_time:=true -p output_file:="$FLEET_DATA_FILE"
  DATA_PID="$STARTED_PID"
  write_run_event data_collection_started "pid=$DATA_PID"
  sleep 2
  kill -0 "$DATA_PID" 2>/dev/null || fail 'Early data collector exited during startup'
fi

spawn_robot() {
  local robot="$1" x="$2" y="$3" yaw="$4" keep_sensors="$5" log_file="$LOG_DIR/$1.log"
  echo "Starting $robot at x=$x y=$y yaw=$yaw..."
  start_group "$log_file" ros2 launch sih_amr_fleet spawn_minimal_amr.launch.py \
    namespace:="$robot" model:="$MODEL" world:="$WORLD_NAME" x:="$x" y:="$y" z:=0.05 yaw:="$yaw" \
    spawn_dock:=false keep_sensors_system:="$keep_sensors" \
    sensor_profile:="$SENSOR_PROFILE" lidar_update_rate_hz:="$LIDAR_UPDATE_RATE_HZ" \
    control_config:="$CONTROL_CONFIG"
  ROBOT_PIDS+=("$STARTED_PID")
  write_run_event robot_launch_started "$robot pid=$STARTED_PID"
  wait_for entity "$SPAWN_WAIT_SECONDS" model_exists "$robot" || fail "$robot body was not created"
  wait_for controller 120 grep -Fq "[$robot.diffdrive_spawner]: Configured and activated diffdrive_controller" "$log_file" || fail "$robot controller did not activate"
  wait_for interfaces 90 grep -Fq "[$robot.interface_readiness]: Interface readiness passed:" "$log_file" || fail "$robot interfaces are not ready"
  echo "$robot passed all gates; settling for ${SETTLE_SECONDS}s."
  write_run_event robot_interface_gate_passed "$robot"
  sleep "$SETTLE_SECONDS"
}

# The Sensors system is loaded once at world scope in warehouse_clean.sdf.
# Per-model copies race over one rendering scene and crash Gazebo.
spawn_robot robot_1 "$ROBOT_1_X" "$ROBOT_1_Y" "$ROBOT_1_YAW" false
spawn_robot robot_2 "$ROBOT_2_X" "$ROBOT_2_Y" "$ROBOT_2_YAW" false
spawn_robot robot_3 "$ROBOT_3_X" "$ROBOT_3_Y" "$ROBOT_3_YAW" false
spawn_robot robot_4 "$ROBOT_4_X" "$ROBOT_4_Y" "$ROBOT_4_YAW" false

if [[ "$START_FLEET" == true ]]; then
  declare -a FLEET_SCENARIO_ARGS=()
  FLEET_LAUNCH_RECORD_DATA="$FLEET_RECORD_DATA"
  [[ -n "$DATA_PID" ]] && FLEET_LAUNCH_RECORD_DATA=false
  [[ -n "$FLEET_SCENARIO_FILE" ]] && FLEET_SCENARIO_ARGS+=(scenario_file:="$FLEET_SCENARIO_FILE")
  start_group "$LOG_DIR/fleet.log" ros2 launch sih_amr_fleet fleet.launch.py \
    random_tasks:="$FLEET_RANDOM_TASKS" record_data:="$FLEET_LAUNCH_RECORD_DATA" \
    data_file:="$FLEET_DATA_FILE" path_tracking_speed_mps:="$FLEET_TRACKING_SPEED_MPS" \
    reservation_time_slot_s:="$FLEET_RESERVATION_SLOT_S" \
    enable_faults:="${FLEET_ENABLE_FAULTS:-false}" \
    enable_spawner:="${FLEET_ENABLE_SPAWNER:-false}" \
    enable_vision:="${FLEET_ENABLE_VISION:-false}" \
    random_seed:="${FLEET_RANDOM_SEED:-42}" \
    "${FLEET_SCENARIO_ARGS[@]}"
  FLEET_PID="$STARTED_PID"
  write_run_event fleet_launch_started "pid=$FLEET_PID"
  sleep 3
  kill -0 "$FLEET_PID" 2>/dev/null || fail 'Fleet launch exited during startup'
fi
if [[ "$START_GUI" == true ]]; then
  # Always use a known-good config rather than the mutable ~/.gz GUI layout.
  # A short retry recovers from a transient EGL / Qt startup failure.
  for attempt in 1 2; do
    start_group "$LOG_DIR/gazebo_gui_attempt_${attempt}.log" gz sim -g \
      --render-engine "$GUI_RENDER_ENGINE" --gui-config "$GUI_CONFIG"
    GUI_PID="$STARTED_PID"
    sleep 4
    kill -0 "$GUI_PID" 2>/dev/null && break
    echo "WARNING: Gazebo GUI exited during attempt $attempt; retrying..." >&2
    GUI_PID=""
  done
  [[ -n "$GUI_PID" ]] || fail 'Gazebo GUI could not start after two attempts'
fi

echo 'All four AMRs passed. Keep this terminal open; Ctrl+C stops the whole run.'
[[ -n "$GUI_PID" ]] && wait "$GUI_PID" || wait "$SERVER_PID"

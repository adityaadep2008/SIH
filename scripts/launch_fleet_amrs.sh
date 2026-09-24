#!/usr/bin/env bash
# Start a clean warehouse, then insert N TurtleBot 4 AMRs sequentially (default 8).
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
SIMULATION_BACKEND="${SIMULATION_BACKEND:-kinematic_lidar_carrier}"
FLEET_COUNT="${FLEET_COUNT:-6}"
SENSOR_PROFILE="${SENSOR_PROFILE:-fleet}"
LIDAR_UPDATE_RATE_HZ="${LIDAR_UPDATE_RATE_HZ:-5.0}"
SIM_RTF_LIMIT="${SIM_RTF_LIMIT:-}"
GZ_SIM_UPDATE_RATE_HZ="${GZ_SIM_UPDATE_RATE_HZ:-}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
GUI_RENDER_ENGINE="${GUI_RENDER_ENGINE:-ogre2}"
GUI_CONFIG="${GZ_GUI_CONFIG:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config}"
START_GUI="${START_GUI:-false}"
START_CHARGING="${START_CHARGING:-false}"
START_FLEET="${START_FLEET:-true}"
CONTROL_CONFIG="${SIH_CONTROL_CONFIG:-$SIH_ROOT/src/sih_amr_fleet/config/fleet_fast_control.yaml}"
export SIH_CONTROL_CONFIG="$CONTROL_CONFIG"
FLEET_RANDOM_TASKS="${FLEET_RANDOM_TASKS:-true}"
FLEET_RECORD_DATA="${FLEET_RECORD_DATA:-true}"
FLEET_SCENARIO_FILE="${FLEET_SCENARIO_FILE:-}"
# Tracking speed profile; default 0.46 m/s (physical_max test point)
FLEET_TRACKING_SPEED_MPS="${FLEET_TRACKING_SPEED_MPS:-0.46}"
FLEET_RESERVATION_SLOT_S="${FLEET_RESERVATION_SLOT_S:-0.0}"
SPAWN_WAIT_SECONDS="${SPAWN_WAIT_SECONDS:-180}"
SETTLE_SECONDS="${SETTLE_SECONDS:-1}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$WORKSPACE/log/fleet_runs/$RUN_ID}"
FLEET_DATA_FILE="${FLEET_DATA_FILE:-$LOG_DIR/fleet_telemetry.jsonl}"
RUN_EVENTS_FILE="$LOG_DIR/run_events.jsonl"

[[ -f "$POSE_FILE" ]] || { echo "ERROR: pose file not found: $POSE_FILE" >&2; exit 2; }
source "$POSE_FILE"
for robot in $(seq 1 "$FLEET_COUNT"); do
  for axis in X Y YAW; do
    variable="ROBOT_${robot}_${axis}"
    [[ -n "${!variable:-}" ]] || { echo "ERROR: set $variable in $POSE_FILE" >&2; exit 2; }
    export "$variable"
  done
done
export FLEET_COUNT

[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 2; }
[[ -f "$GUI_CONFIG" ]] || { echo "ERROR: GUI config not found: $GUI_CONFIG" >&2; exit 2; }
mkdir -p "$LOG_DIR" || { echo "ERROR: cannot create $LOG_DIR" >&2; exit 1; }
write_run_event() {
  printf '{"event_type":"%s","wall_epoch_s":%s,"detail":"%s"}\n' "$1" "$(date +%s)" "$2" >> "$RUN_EVENTS_FILE"
}
write_run_event launcher_started "fleet_count=$FLEET_COUNT headless_or_gui_run_requested"
write_run_event simulation_profile "sensor_profile=$SENSOR_PROFILE lidar_hz=$LIDAR_UPDATE_RATE_HZ tracking_mps=$FLEET_TRACKING_SPEED_MPS target_rtf=${SIM_RTF_LIMIT:-unthrottled}"

source /opt/ros/jazzy/setup.bash
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
elif [[ -f "$OVERLAY/setup.bash" ]]; then
  source "$OVERLAY/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION="${SIH_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_zenoh* ]]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS RMW_FASTRTPS_PUBLICATION_MODE CYCLONEDDS_URI
  export ZENOH_SESSION_CONFIG_URI="${SIH_ZENOH_CONFIG:-$SIH_ROOT/src/sih_amr_fleet/config/zenoh_session_config.json5}"
elif [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps* ]]; then
  unset CYCLONEDDS_URI ZENOH_SESSION_CONFIG_URI
  export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-SYNCHRONOUS}"
  if [[ -n "${SIH_FASTDDS_PROFILE:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SIH_FASTDDS_PROFILE"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  fi
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS RMW_FASTRTPS_PUBLICATION_MODE ZENOH_SESSION_CONFIG_URI
  export CYCLONEDDS_URI="${SIH_CYCLONEDDS_URI:-file://$SIH_ROOT/src/sih_amr_fleet/config/cyclonedds.xml}"
fi
write_run_event middleware_selected "$RMW_IMPLEMENTATION"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM=xcb
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIH_ROOT/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

SERVER_PID="" CLOCK_PID="" GUI_PID="" FLEET_PID="" DATA_PID="" STARTED_PID="" CARRIER_PID="" ZENOHD_PID=""
declare -a ROBOT_PIDS=() CHARGING_PIDS=()

# Start Zenoh router daemon if running rmw_zenoh_cpp and no router is active
if [[ "$RMW_IMPLEMENTATION" == rmw_zenoh* ]]; then
  if ! pgrep -f "rmw_zenohd" >/dev/null 2>&1; then
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$LOG_DIR/rmw_zenohd.log" 2>&1 &
    ZENOHD_PID=$!
    write_run_event zenoh_router_started "pid=$ZENOHD_PID"
    sleep 0.5
  fi
fi

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
  echo 'Stopping this fleet run...'
  for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "$CARRIER_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID" "$ZENOHD_PID"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 10); do
    local alive=false
    for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "$CARRIER_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID" "$ZENOHD_PID"; do
      [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null && alive=true
    done
    [[ "$alive" == false ]] && break
    sleep 0.5
  done
  for pid in "$GUI_PID" "$FLEET_PID" "$DATA_PID" "$CARRIER_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID" "$ZENOHD_PID"; do
    [[ -n "$pid" ]] && kill -KILL "$pid" 2>/dev/null || true
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null || true
  done
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
GZ_SERVER_ARGS=(gz sim -s -r --render-engine "$RENDER_ENGINE")
if [[ -n "$GZ_SIM_UPDATE_RATE_HZ" ]]; then
  [[ "$GZ_SIM_UPDATE_RATE_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 'GZ_SIM_UPDATE_RATE_HZ must be a positive numeric value.'
  echo "Starting Gazebo server with $RENDER_ENGINE at ${GZ_SIM_UPDATE_RATE_HZ} Hz wall-clock update rate (target RTF: ${SIM_RTF_LIMIT:-unspecified})..."
  GZ_SERVER_ARGS+=(-z "$GZ_SIM_UPDATE_RATE_HZ")
else
  echo "Starting unthrottled Gazebo server with $RENDER_ENGINE..."
fi
start_group "$LOG_DIR/gazebo_server.log" "${GZ_SERVER_ARGS[@]}" "$WORLD_FILE"
SERVER_PID="$STARTED_PID"
write_run_event gazebo_server_started "pid=$SERVER_PID"
wait_for server 60 kill -0 "$SERVER_PID" || fail 'Gazebo server exited during startup'
wait_for warehouse 60 warehouse_ready || fail 'Gazebo did not expose the warehouse models'

start_group "$LOG_DIR/clock_bridge.log" ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
CLOCK_PID="$STARTED_PID"
sleep 2
kill -0 "$CLOCK_PID" 2>/dev/null || fail 'Clock bridge exited'

if [[ "$SIMULATION_BACKEND" == "kinematic_lidar_carrier" ]]; then
  echo "Starting Kinematic LiDAR Carrier motion backend..."
  start_group "$LOG_DIR/kinematic_carrier.log" ros2 run sih_amr_fleet kinematic_carrier_node --ros-args -p use_sim_time:=true
  CARRIER_PID="$STARTED_PID"
  write_run_event kinematic_carrier_started "pid=$CARRIER_PID"
  sleep 2
  kill -0 "$CARRIER_PID" 2>/dev/null || fail 'Kinematic carrier node exited during startup'
fi

spawn_robot() {
  local robot="$1" x="$2" y="$3" yaw="$4" keep_sensors="$5" log_file="$LOG_DIR/$1.log"
  echo "Starting $robot at x=$x y=$y yaw=$yaw..."
  if [[ "$SIMULATION_BACKEND" == "kinematic_lidar_carrier" ]]; then
    start_group "$log_file" ros2 launch sih_amr_fleet spawn_carrier_amr.launch.py \
      namespace:="$robot" world:="$WORLD_NAME" x:="$x" y:="$y" z:=0.03 yaw:="$yaw" \
      lidar_update_rate_hz:="$LIDAR_UPDATE_RATE_HZ"
  else
    start_group "$log_file" ros2 launch sih_amr_fleet spawn_minimal_amr.launch.py \
      namespace:="$robot" model:="$MODEL" world:="$WORLD_NAME" x:="$x" y:="$y" z:=0.03 yaw:="$yaw" \
      spawn_dock:=false keep_sensors_system:="$keep_sensors" \
      sensor_profile:="$SENSOR_PROFILE" lidar_update_rate_hz:="$LIDAR_UPDATE_RATE_HZ" \
      control_config:="$CONTROL_CONFIG"
  fi
  ROBOT_PIDS+=("$STARTED_PID")
  write_run_event robot_launch_started "$robot pid=$STARTED_PID"
  wait_for entity "$SPAWN_WAIT_SECONDS" model_exists "$robot" || fail "$robot body was not created"
  if [[ "$SIMULATION_BACKEND" != "kinematic_lidar_carrier" ]]; then
    wait_for controller 120 grep -Fq "[$robot.diffdrive_spawner]: Configured and activated diffdrive_controller" "$log_file" || fail "$robot controller did not activate"
  fi
  wait_for interfaces 90 grep -Fq "[$robot.interface_readiness]: Interface readiness passed:" "$log_file" || fail "$robot interfaces are not ready"
  echo "Verifying $robot physical coordinates and orientation in Gazebo..."
  python3 "$SCRIPT_DIR/verify_gazebo_pose.py" --robot "$robot" --expected-x "$x" --expected-y "$y" --expected-yaw "$yaw" --timeout 15.0 || fail "$robot failed Gazebo physical pose verification"
  echo "$robot passed all gates (including Gazebo pose verification); settling for ${SETTLE_SECONDS}s."
  write_run_event robot_interface_gate_passed "$robot"
  sleep "$SETTLE_SECONDS"
}

# Sequentially spawn fleet members onto their dedicated charging docks
for r in $(seq 1 "$FLEET_COUNT"); do
  x_var="ROBOT_${r}_X"
  y_var="ROBOT_${r}_Y"
  yaw_var="ROBOT_${r}_YAW"
  spawn_robot "robot_${r}" "${!x_var}" "${!y_var}" "${!yaw_var}" false
done


# Verify all fleet members simultaneously in Gazebo before releasing control
echo "Verifying all $FLEET_COUNT AMRs at dock locations in Gazebo..."
for r in $(seq 1 "$FLEET_COUNT"); do
  x_var="ROBOT_${r}_X"
  y_var="ROBOT_${r}_Y"
  yaw_var="ROBOT_${r}_YAW"
  python3 "$SCRIPT_DIR/verify_gazebo_pose.py" --robot "robot_${r}" --expected-x "${!x_var}" --expected-y "${!y_var}" --expected-yaw "${!yaw_var}" --tol-xy 0.45 --tol-yaw 0.75 --timeout 15.0 || fail "robot_${r} failed final Gazebo pre-launch check"
done
echo "All $FLEET_COUNT AMRs successfully verified at correct coordinates and angles in Gazebo."

if [[ "$START_FLEET" == true ]]; then
  declare -a FLEET_SCENARIO_ARGS=()
  [[ -n "$FLEET_SCENARIO_FILE" ]] && FLEET_SCENARIO_ARGS+=(scenario_file:="$FLEET_SCENARIO_FILE")
  start_group "$LOG_DIR/fleet.log" ros2 launch sih_amr_fleet fleet.launch.py \
    random_tasks:="$FLEET_RANDOM_TASKS" record_data:="$FLEET_RECORD_DATA" \
    data_file:="$FLEET_DATA_FILE" path_tracking_speed_mps:="$FLEET_TRACKING_SPEED_MPS" \
    reservation_time_slot_s:="$FLEET_RESERVATION_SLOT_S" \
    num_robots:="$FLEET_COUNT" \
    consolidated:=true \
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
  gz sim -g --render-engine "$GUI_RENDER_ENGINE" --gui-config "$GUI_CONFIG" &
  GUI_PID=$!
fi

echo "All $FLEET_COUNT AMRs passed bring-up. Telemetry logging active to $FLEET_DATA_FILE."
[[ -n "$GUI_PID" ]] && wait "$GUI_PID" || wait "$SERVER_PID"

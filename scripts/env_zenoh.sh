#!/usr/bin/env bash
# ==============================================================================
# SIH Multi-AMR Fleet - Zenoh Environment Activator
# Sets active shell to Eclipse Zenoh (rmw_zenoh_cpp) with clean router topology
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIH_SRC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$SIH_SRC_DIR/../.." && pwd)"
ZENOH_CONFIG_PATH="$SIH_SRC_DIR/src/sih_amr_fleet/config/zenoh_session_config.json5"

source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG_PATH"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "[ZENOH] Active RMW Middleware: $RMW_IMPLEMENTATION"
echo "[ZENOH] Session Config: $ZENOH_SESSION_CONFIG_URI"
echo "[ZENOH] Discovery Range: $ROS_AUTOMATIC_DISCOVERY_RANGE"

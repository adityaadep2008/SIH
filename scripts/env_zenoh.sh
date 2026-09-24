#!/usr/bin/env bash
# ==============================================================================
# Universal ROS 2 Jazzy & Eclipse Zenoh Environment Activation
# Supports both bash and zsh shells
# ==============================================================================

# Detect directory containing this script
if [ -n "${ZSH_VERSION:-}" ]; then
  CURRENT_SCRIPT_DIR="${0:A:h}"
else
  CURRENT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

SIH_ROOT="$(cd "$CURRENT_SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$SIH_ROOT/../.." && pwd)"
ZENOH_CONFIG_PATH="$SIH_ROOT/src/sih_amr_fleet/config/zenoh_mesh_peer.json5"

# 1. Source ROS 2 Jazzy
if [ -n "${ZSH_VERSION:-}" ] && [ -f "/opt/ros/jazzy/setup.zsh" ]; then
  source "/opt/ros/jazzy/setup.zsh"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source "/opt/ros/jazzy/setup.bash"
fi

# 2. Source Workspace Underlay
if [ -n "${ZSH_VERSION:-}" ] && [ -f "$WORKSPACE_ROOT/install/setup.zsh" ]; then
  source "$WORKSPACE_ROOT/install/setup.zsh"
elif [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi

# 3. Export Zenoh Middleware Configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG_PATH"
export ZENOH_CONFIG_FILE="$ZENOH_CONFIG_PATH"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "[ZENOH] Middleware: $RMW_IMPLEMENTATION"
echo "[ZENOH] Session Config: $ZENOH_SESSION_CONFIG_URI"


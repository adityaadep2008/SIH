#!/usr/bin/env bash
# ==============================================================================
# SIH Multi-AMR Fleet - Eclipse Zenoh Middleware Setup & Activation Script
# Target: ROS 2 Jazzy Jalisco on Ubuntu 24.04 (Noble)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIH_SRC_DIR="$WORKSPACE_ROOT/src/SIH"
ZENOH_CONFIG_PATH="$SIH_SRC_DIR/src/sih_amr_fleet/config/zenoh_mesh_peer.json5"

echo "=== [1/4] Checking ROS 2 Jazzy Environment ==="
if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "/opt/ros/jazzy/setup.bash"
  else
    echo "[ERROR] ROS 2 Jazzy is not sourced or installed in /opt/ros/jazzy."
    exit 1
  fi
fi
echo "Detected ROS_DISTRO: $ROS_DISTRO"

echo "=== [2/4] Installing rmw_zenoh_cpp ==="
if dpkg -l | grep -q "ros-jazzy-rmw-zenoh-cpp"; then
  echo "[OK] ros-jazzy-rmw-zenoh-cpp is already installed."
else
  echo "[INFO] Installing ros-jazzy-rmw-zenoh-cpp via apt..."
  sudo apt update -y
  sudo apt install -y ros-jazzy-rmw-zenoh-cpp
  echo "[OK] Installed ros-jazzy-rmw-zenoh-cpp successfully."
fi

echo "=== [3/4] Verifying Zenoh Mesh Peer Configuration ==="
if [ -f "$ZENOH_CONFIG_PATH" ]; then
  echo "[OK] Found Zenoh Mesh config at: $ZENOH_CONFIG_PATH"
else
  echo "[ERROR] Zenoh config file missing at $ZENOH_CONFIG_PATH"
  exit 1
fi

echo "=== [4/4] Generating Activation Environment Helper ==="
ENV_HELPER="$SIH_SRC_DIR/scripts/env_zenoh.sh"
cat <<EOF > "$ENV_HELPER"
#!/usr/bin/env bash
# Source this file to switch active shell to Zenoh Peer-to-Peer Middleware
source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi
# 4. Export Zenoh Middleware Configuration
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG_PATH"
export ZENOH_CONFIG_FILE="$ZENOH_CONFIG_PATH"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
echo "[ZENOH] Active RMW: \$RMW_IMPLEMENTATION"
echo "[ZENOH] Peer-to-Peer Mesh Config: \$ZENOH_SESSION_CONFIG_URI"
EOF

chmod +x "$ENV_HELPER"
chmod +x "$SCRIPT_DIR/setup_zenoh.sh"

echo ""
echo "=============================================================================="
echo " [SUCCESS] Eclipse Zenoh Setup Complete!"
echo " To activate Zenoh in any terminal session, run:"
echo "   source $SIH_SRC_DIR/scripts/env_zenoh.sh"
echo "=============================================================================="

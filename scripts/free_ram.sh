#!/usr/bin/env bash
# ==============================================================================
# AM-CORD AI | Fast Emergency Memory Reclamation & Process Cleanup Utility
# ==============================================================================
# Safely reclaims memory without requiring a full machine reboot if Firefox,
# browser content processes, or background nodes ever hang or thrash memory.
# ==============================================================================

set -euo pipefail

echo "========================================================"
echo " AM-CORD AI | Memory Reclamation & Cleanup Utility"
echo "========================================================"
echo " Initial Memory State:"
free -h
echo "--------------------------------------------------------"

echo "[1/4] Terminating browser processes (Firefox / snap)..."
pkill -9 -f firefox 2>/dev/null || true
pkill -9 -f firefox-bin 2>/dev/null || true
pkill -9 -f "Isolated Web Co" 2>/dev/null || true
pkill -9 -f glxtest 2>/dev/null || true

echo "[2/4] Terminating orphaned bridge server instances..."
pkill -f "scripts/dashboard_bridge_server.py" 2>/dev/null || true

echo "[3/4] Cleaning stale IPC shared memory in /dev/shm..."
find /dev/shm/ -maxdepth 1 \( -name "*fastrtps*" -o -name "*shm-event*" -o -name "*lttng*" \) -delete 2>/dev/null || true

echo "[4/4] Syncing filesystem buffers & dropping reclaimable caches..."
sync
if [ "$(id -u)" -eq 0 ]; then
    echo 3 > /proc/sys/vm/drop_caches 2>/dev/null || true
elif sudo -n true 2>/dev/null; then
    echo 3 | sudo tee /proc/sys/vm/drop_caches >/dev/null 2>&1 || true
else
    echo "  (Note: Run with sudo to force kernel pagecache purge)"
fi

echo "--------------------------------------------------------"
echo " Final Memory State:"
free -h
echo "========================================================"
echo "✔ Memory reclamation complete."

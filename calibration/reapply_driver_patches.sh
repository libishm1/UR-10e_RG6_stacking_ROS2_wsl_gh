#!/bin/bash
# Re-apply vendor-tree patches that git can't track because the vendor
# packages are sub-gits managed by `vcs import`.
#
# Run this after every fresh `vcs import src < ros2.repos` (especially with
# --force or after a clean clone).
#
# Patches applied:
#   1. ur_robot_driver/config/ur10e_update_rate.yaml: 500 Hz → 250 Hz
#      Reduces RTDE pipeline overflow rate on WSL2 (non-RT kernel).
#      Cabinet still publishes at 500 Hz but with non_blocking_read=true
#      the driver drops samples instead of buffering. Steady-state overflow
#      drops from spam to ~8/s. See wiki/known_bugs_and_workarounds.md
#      entry "RTDE Pipeline producer overflowed spam on WSL2".

set -e

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UPDATE_RATE_YAML="$WS_ROOT/src/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur10e_update_rate.yaml"

if [ ! -f "$UPDATE_RATE_YAML" ]; then
    echo "ERROR: $UPDATE_RATE_YAML not found"
    echo "Run 'vcs import src < ros2.repos' first."
    exit 1
fi

echo "=== Patching $UPDATE_RATE_YAML (500 Hz -> 250 Hz) ==="
# Idempotent: only write if not already at 250
if grep -q "update_rate: 250" "$UPDATE_RATE_YAML"; then
    echo "  already at 250 Hz; nothing to do."
else
    sed -i.bak 's/^\s*update_rate:\s*500\b/    update_rate: 250  # patched by calibration\/reapply_driver_patches.sh/' "$UPDATE_RATE_YAML"
    if grep -q "update_rate: 250" "$UPDATE_RATE_YAML"; then
        echo "  done."
    else
        echo "  PATCH FAILED — file content unexpected. See backup at ${UPDATE_RATE_YAML}.bak"
        exit 2
    fi
fi

echo
echo "Re-build with: colcon build --packages-select ur_robot_driver --symlink-install"

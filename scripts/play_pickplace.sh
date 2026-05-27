#!/bin/bash
# Run the pick-and-place test script against whatever stack is currently up.
#
# What this does:
#   1. Check that a stack is running (move_group must be alive); fail with
#      a clear message if not. We DO NOT kill ROS here — that would
#      destroy whatever stack the user just started (sim or real).
#   2. Source ROS 2 Humble + workspace overlay
#   3. Run tests/play_pickplace.py with the supplied args (forwarded as-is)
#
# Hardware required:
#   * A stack is already up (started via either scripts/launch_sim.sh
#     or scripts/launch_real.sh)
#   * For real-hw: external_control.urp must be PLAYING on the pendant
#     with Remote Control mode on
#
# Usage:
#   bash scripts/play_pickplace.sh                          # all 20 cycles, sim gripper
#   bash scripts/play_pickplace.sh --max 4                  # 2 full cycles
#   bash scripts/play_pickplace.sh --max 4 --real-gripper   # with real gripper (real-hw only)
#   bash scripts/play_pickplace.sh --help                   # full arg list

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

# --- 1. require move_group to be running ---
if ! pgrep -f "moveit_ros_move_group/lib/moveit_ros_move_group/move_group" >/dev/null; then
  echo "!! move_group is NOT running."
  echo "   Start the stack first with one of:"
  echo "     bash scripts/launch_sim.sh   # simulation"
  echo "     bash scripts/launch_real.sh  # real cabinet"
  exit 1
fi

# --- 2. source environment ---
source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi

# --- 3. forward all args to the python script ---
echo ">> python3 tests/play_pickplace.py $@"
cd "$WS_ROOT"
exec python3 tests/play_pickplace.py "$@"

#!/bin/bash
# Launch the full UR10e + RG6 + MoveIt stack in SIMULATION (fake hardware).
#
# What this does:
#   1. Kill any leftover ROS processes (prevents two RViz windows, port
#      conflicts on controller_manager, stale move_group, etc.)
#   2. Source ROS 2 Humble + workspace overlay
#   3. Launch full_stack.launch.py with use_fake_hardware:=true
#   4. Print PIDs so you can verify the launch came up
#
# Hardware required: none. RViz opens with the URDF model; mock_components
# stands in for the cabinet and accepts trajectories without moving anything
# physical.
#
# After this returns, the stack is running in the background. Test with:
#   python3 tests/play_pickplace.py --max 4
#
# To stop everything:
#   bash scripts/kill_ros.sh
#
# Usage:
#   bash scripts/launch_sim.sh
#
# Logs go to /tmp/full_stack.log.

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

# --- 1. kill anything previously running ---
echo ">> Killing any leftover ROS processes (no duplicate RViz!)"
bash "$HERE/kill_ros.sh"

# --- 2. source environment ---
source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
else
  echo "!! $WS_ROOT/install/setup.bash not found — did you 'colcon build' yet?"
  exit 1
fi

# --- 3. launch (background, detached, redirect to log) ---
rm -f /tmp/full_stack.log
echo ">> ros2 launch ur10e_rg6_moveit_config full_stack.launch.py use_fake_hardware:=true"
setsid nohup ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
  use_fake_hardware:=true \
  > /tmp/full_stack.log 2>&1 < /dev/null &
disown

# --- 4. wait for the slowest piece (move_group; usually ~15 s) ---
echo ">> Waiting up to 25 s for move_group to come up..."
for i in $(seq 1 25); do
  sleep 1
  if pgrep -f "moveit_ros_move_group/lib/moveit_ros_move_group/move_group" >/dev/null; then
    echo "   move_group up after ${i}s."
    break
  fi
done

echo ""
echo ">> Stack processes alive:"
pgrep -af "move_group|ros2_control_node|robot_state_publisher|rviz2" | grep -v "pgrep " | head -6
echo ""
echo ">> Log tail (any errors after the 'You can start planning now!' line):"
tail -5 /tmp/full_stack.log 2>/dev/null | grep -v "Pipeline producer overflowed" | head -5

echo ""
echo "Stack ready. Run a test with:"
echo "   python3 tests/play_pickplace.py --max 4"
echo "Stop with:"
echo "   bash scripts/kill_ros.sh"

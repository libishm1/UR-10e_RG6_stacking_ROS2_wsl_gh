#!/bin/bash
# Kill every ROS-related process from this workspace.
#
# Why a script file (not inline `pkill`):
#   pkill -f matches against /proc/PID/cmdline. If the calling shell's
#   own command line contains the pattern (e.g. "ros2 launch"), pkill
#   kills the shell first and the script aborts with exit 9 (SIGKILL).
#   Storing patterns in this file and invoking via `bash scripts/kill_ros.sh`
#   keeps the patterns OUT of the caller's argv → no self-kill.
#
# Safe to run any time — idempotent, prints what (if anything) survived.
#
# Usage:
#   bash scripts/kill_ros.sh

PATTERNS=(
  "ros2 launch"
  "ur_ros2_control_node"
  "ros2_control_node"
  "controller_manager"
  "move_group"
  "rviz2"
  "robot_state_publisher"
  "spawner --controller-manager"
  "robot_state_helper"
  "dashboard_client"
  "urscript_interface"
  "tool_communication"
  "/tmp/ttyUR"
)
for pat in "${PATTERNS[@]}"; do
  pkill -9 -f "$pat" >/dev/null 2>&1 || true
done

sleep 2

echo "--- alive after kill ---"
pgrep -af "ur_ros2_control_node|controller_manager|move_group|rviz2|robot_state_publisher|ros2_control_node" 2>/dev/null || true
pgrep -af "ros2 launch" 2>/dev/null || true
echo "(end)"
exit 0

#!/bin/bash
# Launch the full UR10e + RG6 + MoveIt stack against REAL HARDWARE.
#
# What this does:
#   1. Kill any leftover ROS processes (prevents two RViz windows, port
#      conflicts on controller_manager, stale move_group, etc.)
#   2. Ping the cabinet (fast-fail if the network is down)
#   3. Source ROS 2 Humble + workspace overlay
#   4. Launch full_stack.launch.py with use_fake_hardware:=false and
#      robot_ip:=192.168.1.100 (the verified cabinet IP for this cell)
#   5. Print PIDs and connection state so you can verify the launch
#
# Hardware required:
#   * UR10e powered on, in RUNNING / NORMAL safety state
#   * Cabinet reachable at 192.168.1.100 (verified cell config — see
#     reference memory ur10e_cell_network)
#   * Windows Firewall inbound 50001–50004 from cabinet IP enabled
#     (one-time setup; see SESSION_HANDOFF 2026-05-26 evening checkpoint)
#   * External Control URCap installed on cabinet (one-time setup)
#
# After this returns, the driver is connected to the cabinet but the
# arm WILL NOT MOVE until you also load+Play `external_control.urp` on
# the pendant with Remote Control mode on. RTDE state readback (joints,
# TCP) is live regardless.
#
# To stop everything:
#   bash scripts/kill_ros.sh
#
# Usage:
#   bash scripts/launch_real.sh
#   bash scripts/launch_real.sh 192.168.1.123     # override robot IP
#
# Logs go to /tmp/full_stack.log.

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

ROBOT_IP="${1:-192.168.1.100}"

# --- 1. kill anything previously running ---
echo ">> Killing any leftover ROS processes (no duplicate RViz!)"
bash "$HERE/kill_ros.sh"

# --- 2. ping the cabinet to fast-fail if network is down ---
echo ">> Verifying cabinet reachable at $ROBOT_IP ..."
if ! ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "!! Cabinet at $ROBOT_IP is NOT reachable."
  echo "   Check: cabinet power, Ethernet cable, this laptop's IP"
  echo "   (should be 192.168.1.35 on the cell subnet)."
  exit 1
fi
echo "   ping OK."

# --- 3. quick TCP probe of Dashboard + RTDE (sanity) ---
if ! timeout 3 bash -c "</dev/tcp/$ROBOT_IP/29999" 2>/dev/null; then
  echo "!! Dashboard port 29999 not reachable on cabinet $ROBOT_IP."
  exit 1
fi
if ! timeout 3 bash -c "</dev/tcp/$ROBOT_IP/30004" 2>/dev/null; then
  echo "!! RTDE port 30004 not reachable on cabinet $ROBOT_IP."
  exit 1
fi
echo "   Dashboard (29999) + RTDE (30004) OK."

# --- 4. source environment ---
source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
else
  echo "!! $WS_ROOT/install/setup.bash not found — did you 'colcon build' yet?"
  exit 1
fi

# --- 5. launch (background, detached, redirect to log) ---
rm -f /tmp/full_stack.log
echo ">> ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \\"
echo "      use_fake_hardware:=false robot_ip:=$ROBOT_IP"
setsid nohup ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:="$ROBOT_IP" \
  > /tmp/full_stack.log 2>&1 < /dev/null &
disown

# --- 6. wait for the slowest piece (move_group; ~18 s on real-hw) ---
echo ">> Waiting up to 30 s for move_group to come up..."
for i in $(seq 1 30); do
  sleep 1
  if pgrep -f "moveit_ros_move_group/lib/moveit_ros_move_group/move_group" >/dev/null; then
    echo "   move_group up after ${i}s."
    break
  fi
done

echo ""
echo ">> Stack processes alive:"
pgrep -af "move_group|ur_ros2_control_node|robot_state_publisher|rviz2" | grep -v "pgrep " | head -6
echo ""
echo ">> Log tail (filtered):"
tail -50 /tmp/full_stack.log 2>/dev/null \
  | grep -v "Pipeline producer overflowed" \
  | grep -E "Robot connected|Configured and activated|ERROR|terminate|external_control" \
  | head -10

echo ""
echo "Driver is connected to the cabinet but the arm will NOT move until:"
echo "   1. On the pendant: load 'external_control.urp', press Play"
echo "   2. On the pendant: top-right → Remote Control mode"
echo ""
echo "Then test motion with:"
echo "   python3 tests/play_pickplace.py --max 4              # motion only"
echo "   python3 tests/play_pickplace.py --max 4 --real-gripper  # with gripper"
echo ""
echo "Stop everything with:"
echo "   bash scripts/kill_ros.sh"

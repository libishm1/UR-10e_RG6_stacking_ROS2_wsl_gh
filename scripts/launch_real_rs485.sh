#!/bin/bash
# Launch the full UR10e + RG6 + MoveIt stack against REAL HARDWARE, WITH the
# OnRobot RG6 driven over the tool-flange RS485 (Modbus) — the chosen gripper
# path. Same as launch_real.sh but enables use_tool_communication so the
# gripper is reachable from ROS via /tmp/ttyUR.
#
# What this does:
#   1. Kill any leftover ROS processes (no duplicate RViz / port clashes)
#   2. Ping + TCP-probe the cabinet (fast-fail if the network is down)
#   3. Source ROS 2 Humble + workspace overlay
#   4. Launch full_stack.launch.py with use_fake_hardware:=false,
#      robot_ip:=192.168.1.100, and the tool RS485 bridge enabled:
#        use_tool_communication:=true  tool_voltage:=24
#        tool_parity:=2 (even)  tool_baud_rate:=1000000
#        tool_stop_bits:=1  tool_rx_idle_chars:=1.5  tool_tx_idle_chars:=3.5
#        tool_device_name:=/tmp/ttyUR
#   5. Print PIDs and whether /tmp/ttyUR appeared.
#
# Pendant prereqs (load the `ros` installation — NOT the OnRobot default):
#   * Tool I/O → Controlled by: User
#   * Tool I/O → Communication Interface (RS485), 1M / Even / One / 1.5 / 3.5
#   * Tool Output Voltage: 24 V
#   * OnRobot Setup → Device: No connection (so the URCap releases RS485)
#   * rs485 daemon URCap installed (CONFIRMED REQUIRED, coexists with OnRobot
#     URCap): /root/.urcaps/rs485-1.0.jar — it opens 54321 / bridges the tool.
#   * Cabinet firewall: inbound port 54321 ALLOWED (Settings → Security →
#     General). Without it the host can't reach the daemon.
#   * external_control.urp must be PLAYING — the rs485 bridge runs inside the
#     control program; the GRIPPER is NOT reachable until the URP plays.
#
# OPERATIONAL TRAPS (see wiki/known_bugs_and_workarounds.md "RG6 over RS485"):
#   * Gripper Modbus needs the URP PLAYING (not just the stack up).
#   * If Modbus reads return None but 54321 is open: the cabinet rs485 daemon
#     is stuck on a stale connection (common after `wsl --shutdown` or
#     admin-mode WSL). Replaying External Control does NOT fix it — RESTART
#     POLYSCOPE. And use a NORMAL (non-admin) WSL terminal.
#   * socat locks the pty after one open/close — open once & hold, or reset
#     socat between standalone test runs.
#
# After this returns, the arm won't move until external_control.urp is Playing
# + Remote Control. Then test the gripper (URP must be playing) with:
#   python3 tests/onrobot_modbus_grip.py status
#   python3 tests/onrobot_modbus_grip.py cycle
#
# To stop everything:  bash scripts/kill_ros.sh
#
# Usage:
#   bash scripts/launch_real_rs485.sh
#   bash scripts/launch_real_rs485.sh 192.168.1.123     # override robot IP
#
# Logs go to /tmp/full_stack.log.

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

ROBOT_IP="${1:-192.168.1.100}"
TOOL_DEVICE="/tmp/ttyUR"

# --- 1. kill anything previously running ---
echo ">> Killing any leftover ROS processes (no duplicate RViz!)"
bash "$HERE/kill_ros.sh"

# --- 1b. require socat (the tool-comm bridge needs it to create /tmp/ttyUR) ---
if ! command -v socat >/dev/null 2>&1; then
  echo "!! 'socat' is not installed — the tool RS485 bridge cannot create"
  echo "   $TOOL_DEVICE without it. Install once with:"
  echo "       sudo apt-get install -y socat"
  exit 1
fi

# --- 2. ping the cabinet ---
echo ">> Verifying cabinet reachable at $ROBOT_IP ..."
if ! ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "!! Cabinet at $ROBOT_IP is NOT reachable."
  echo "   Check: cabinet power, Ethernet cable, this laptop's IP (192.168.1.35)."
  exit 1
fi
echo "   ping OK."

# --- 3. quick TCP probe of Dashboard + RTDE ---
if ! timeout 3 bash -c "</dev/tcp/$ROBOT_IP/29999" 2>/dev/null; then
  echo "!! Dashboard port 29999 not reachable on cabinet $ROBOT_IP."; exit 1
fi
if ! timeout 3 bash -c "</dev/tcp/$ROBOT_IP/30004" 2>/dev/null; then
  echo "!! RTDE port 30004 not reachable on cabinet $ROBOT_IP."; exit 1
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

# Stale device from a previous run would mask a bridge failure; clear it.
rm -f "$TOOL_DEVICE" 2>/dev/null || true

# --- 5. launch (background, detached, redirect to log) ---
rm -f /tmp/full_stack.log
echo ">> ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \\"
echo "      use_fake_hardware:=false robot_ip:=$ROBOT_IP use_tool_communication:=true ..."
setsid nohup ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
  use_fake_hardware:=false \
  robot_ip:="$ROBOT_IP" \
  use_tool_communication:=true \
  tool_voltage:=24 \
  tool_parity:=2 \
  tool_baud_rate:=1000000 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_device_name:="$TOOL_DEVICE" \
  > /tmp/full_stack.log 2>&1 < /dev/null &
disown

# --- 6. wait for move_group ---
echo ">> Waiting up to 30 s for move_group to come up..."
for i in $(seq 1 30); do
  sleep 1
  if pgrep -f "moveit_ros_move_group/lib/moveit_ros_move_group/move_group" >/dev/null; then
    echo "   move_group up after ${i}s."
    break
  fi
done

# --- 7. check the tool RS485 device appeared ---
echo ""
echo ">> Tool RS485 bridge device:"
if [ -e "$TOOL_DEVICE" ]; then
  echo "   $TOOL_DEVICE present — gripper Modbus should be reachable."
else
  echo "   !! $TOOL_DEVICE NOT present. The tool-comm bridge didn't come up."
  echo "      Likely causes (see wiki/rg6_rs485_modbus.md):"
  echo "        - pendant not on the 'ros' installation (Communication Interface +"
  echo "          Controlled by User + OnRobot device None)"
  echo "        - the rs485 daemon URCap may be required on this controller"
  echo "      Check the log: grep -i 'tool\\|rs485\\|socat\\|ttyUR' /tmp/full_stack.log"
fi

echo ""
echo ">> Stack processes alive:"
pgrep -af "move_group|ur_ros2_control_node|robot_state_publisher|rviz2" | grep -v "pgrep " | head -6

echo ""
echo "Gripper (no arm motion needed) — test Modbus now:"
echo "   python3 tests/onrobot_modbus_grip.py status"
echo "   python3 tests/onrobot_modbus_grip.py cycle"
echo ""
echo "Arm motion still needs the pendant: load external_control.urp, Play,"
echo "Remote Control. Then:"
echo "   python3 tests/play_pickplace.py --max 4 --real-gripper"
echo ""
echo "Stop everything with:  bash scripts/kill_ros.sh"

#!/bin/bash
# Bring up RViz with the MoveIt config in the FOREGROUND, from YOUR terminal.
#
# WHY THIS EXISTS:
#   1. The launch scripts (launch_sim.sh / launch_real*.sh) bundle RViz into a
#      DETACHED `setsid nohup ros2 launch ...`. When the stack is started from
#      a one-shot / non-interactive session, that detached RViz window does
#      NOT surface under WSLg. Launching RViz attached, from your own
#      interactive WSL terminal, makes the window appear reliably.
#   2. The RViz Fixed Frame is "world", but the SRDF virtual joint
#      `world_to_base` is a MoveIt planning construct that is NEVER published
#      to TF (robot_state_publisher only knows the URDF, rooted at base_link).
#      So "world" is unresolvable and the robot won't render even if the window
#      shows. This script publishes a static world->base_link TF to fix that.
#
# USAGE (run in your own interactive WSL terminal, AFTER the stack is up):
#   bash scripts/view_rviz.sh
#
# The stack (launch_sim.sh / launch_real*.sh) should already be running so
# /robot_description + joint states + /monitored_planning_scene exist.

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi

# --- WSLg display sanity ---
echo ">> DISPLAY='$DISPLAY'  WAYLAND_DISPLAY='$WAYLAND_DISPLAY'"
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  echo "!! No DISPLAY/WAYLAND_DISPLAY set — WSLg GUI env is missing for this shell."
  echo "   Fix: from Windows PowerShell run 'wsl --shutdown', reopen WSL, retry."
  echo "   (WSLg injects DISPLAY=:0 / WAYLAND_DISPLAY=wayland-0 on a healthy boot.)"
  exit 1
fi

# NOTE: the world->base_link static TF (needed so Fixed Frame "world" resolves)
# is now published by full_stack.launch.py. If you run RViz WITHOUT the full
# stack, either publish it yourself:
#   ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id base_link
# or set RViz → Global Options → Fixed Frame → base_link.

# --- kill any existing RViz so we never end up with TWO windows ---
# full_stack.launch.py bundles its OWN (detached) RViz; this script then runs
# a foreground one that actually surfaces under WSLg. Kill the bundled one
# first so there's exactly ONE window. (Safe: this script's own cmdline is
# "bash .../view_rviz.sh" — it contains "view_rviz", not "rviz2".)
pkill -9 -f "rviz2 -d" >/dev/null 2>&1 || true
sleep 1

# --- RViz config path (install overlay, fall back to src) ---
RVIZ_CFG="$WS_ROOT/install/ur10e_rg6_moveit_config/share/ur10e_rg6_moveit_config/config/moveit.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
  RVIZ_CFG="$WS_ROOT/src/ur10e_rg6_moveit_config/config/moveit.rviz"
fi

echo ">> Launching RViz (foreground) -d $RVIZ_CFG"
echo "   If the robot still doesn't appear: RViz → Global Options → Fixed Frame"
echo "   → set to 'base_link' (or confirm the world->base_link TF above is live)."
exec rviz2 -d "$RVIZ_CFG"

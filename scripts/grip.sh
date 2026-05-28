#!/bin/bash
# Open / close / grip the OnRobot RG6 over RS485 Modbus — STANDALONE, no Claude.
#
# Thin wrapper over tests/onrobot_modbus_grip.py, which auto-swaps a FRESH socat
# on every run (socat's pty locks after one open/close — the 2026-05-28 finding,
# see wiki/rg6_rs485_modbus.md). So you can run open/close as many times as you
# like, each invocation just works.
#
# ── ONE-TIME PER SESSION (bring the gripper online) ──────────────────────────
#   1. bash scripts/launch_real_rs485.sh        # stack + tool RS485 bridge
#   2. On the pendant: load the `ros` installation (Tool I/O Controlled by User
#      + Communication Interface + 24V + OnRobot device None), then Play
#      external_control.urp in Remote Control. The rs485 bridge that carries the
#      gripper Modbus runs INSIDE that control program, so the URP MUST be
#      playing — the gripper is unreachable otherwise.
#
#   If a command says CONNECT FAILED / no response even though 54321 looks open:
#   the cabinet's rs485 daemon is wedged (common after `wsl --shutdown` or an
#   ADMIN-mode WSL session). Replaying External Control does NOT fix it —
#   RESTART POLYSCOPE, redo steps 1-2, and use a NORMAL (non-admin) WSL terminal.
#
# ── USAGE ────────────────────────────────────────────────────────────────────
#   bash scripts/grip.sh open            # open ~150 mm (clear the part)
#   bash scripts/grip.sh close           # close fully / grip object at 40 N
#   bash scripts/grip.sh close 30 60     # close to 30 mm at 60 N
#   bash scripts/grip.sh status          # width + grip_detected
#   bash scripts/grip.sh cycle           # close then open
#
# grip_detected (true/false) is reliable right after a `close`: it reads the
# status word (offset 10) — bit0 = object gripped, bit1 = closed on nothing.
#
# Speeds/forces stay conservative by default (40 N = gentle, per the project's
# safe-default rule). Pass a higher force only when you mean to.

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/.." && pwd)"

# The gripper client is pure pyserial + socat (no ROS needed), but source the
# overlay anyway so it runs from any shell.
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash" 2>/dev/null || true
fi

if ! command -v socat >/dev/null 2>&1; then
  echo "!! 'socat' not installed — run: sudo apt-get install -y socat"
  exit 1
fi

exec python3 "$WS_ROOT/tests/onrobot_modbus_grip.py" "$@"

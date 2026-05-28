# scripts/ — operator launch helpers

These scripts are the supported, no-Claude-Code way to bring the
UR10e + RG6 stack up, drive the gripper, run a pick-and-place test, and tear
everything down. They live in the repo (not /tmp) so they survive WSL restarts.

| Script | What it does | Hardware required |
|---|---|---|
| `launch_sim.sh` | Full MoveIt + ros2_control stack with `use_fake_hardware:=true`. RViz opens; nothing physical moves. | None |
| `launch_real.sh` | Same stack against the cabinet at 192.168.1.100. Pings + TCP-probes before launch. | UR10e powered + on the cell subnet; pendant URP later |
| `launch_real_rs485.sh` | Real stack **plus** the tool-RS485 bridge (`use_tool_communication`) so the RG6 is reachable over Modbus. | UR10e + `ros` installation + rs485 URCap + firewall 54321 |
| `grip.sh` | **Open/close/grip the RG6 over Modbus, standalone.** `grip.sh {open\|close\|status\|cycle}`. Auto-swaps a fresh socat each run. | Stack up via `launch_real_rs485.sh` + URP playing |
| `play_pickplace.sh` | Runs `tests/play_pickplace.py` against whatever stack is already up. | A stack must be running |
| `view_rviz.sh` | Launch RViz attached in your own terminal (surfaces under WSLg) + single window. | A stack must be running |
| `kill_ros.sh` | Hard-kills every ROS process this workspace can spawn (incl. socat). Idempotent. | None |

All scripts auto-source `/opt/ros/humble/setup.bash` and the workspace
overlay at `install/setup.bash`. You do NOT need to source anything
yourself before running them.

---

## Typical flows

### Run a sim test from a fresh shell

```bash
cd ~/ur_rg6_ws
bash scripts/launch_sim.sh                 # opens RViz, waits for move_group
bash scripts/play_pickplace.sh --max 4     # 2 full pick-place cycles
bash scripts/kill_ros.sh                   # tear down when done
```

### Drive the RG6 gripper standalone (RS485/Modbus)

```bash
cd ~/ur_rg6_ws
bash scripts/launch_real_rs485.sh          # stack + tool-RS485 bridge (once)
# Pendant: load the `ros` installation, Play external_control.urp (Remote Control)
bash scripts/grip.sh open                  # open ~150 mm
bash scripts/grip.sh close                 # close / grip object at 40 N
bash scripts/grip.sh status                # width + grip_detected
```
Run `grip.sh open`/`close` as often as you like — it swaps a fresh socat each
run, so the pty-lock never bites. If a command says CONNECT FAILED with no
response, the cabinet's rs485 daemon is wedged (e.g. after `wsl --shutdown` or
admin-mode WSL) → **restart PolyScope**, relaunch, and use a non-admin WSL
terminal. Details: [`../wiki/rg6_rs485_modbus.md`](../wiki/rg6_rs485_modbus.md).

### Run on real hardware

1. Power the cabinet, confirm laptop is at `192.168.1.35` on the cell
   subnet.
2. Bring the stack up:
   ```bash
   bash scripts/launch_real.sh
   ```
3. On the pendant: load `external_control.urp`, press Play, switch the
   top-right corner to Remote Control mode.  Until this is done the arm
   will NOT move (RTDE state readback works regardless).
4. Run a small motion test (no gripper):
   ```bash
   bash scripts/play_pickplace.sh --max 4
   ```
5. With the real RG6 gripper (binary close/open via Tool I/O — see
   `tests/onrobot_io_grip.py`):
   ```bash
   bash scripts/play_pickplace.sh --max 4 --real-gripper
   ```
   Pendant prereq: Installation → General → Tool I/O →
   `Controlled by: User` (so the OnRobot URCap doesn't fight our writes).
6. Stop:
   ```bash
   bash scripts/kill_ros.sh
   ```

### Override the cabinet IP

```bash
bash scripts/launch_real.sh 192.168.1.123
```

### "Why won't it run? I just launched!"

`play_pickplace.sh` refuses to start if `move_group` isn't alive. It
will NOT kill your running stack — by design, so you don't lose the
state you just brought up. If you see the refusal message, either
`launch_sim.sh` / `launch_real.sh` hasn't been run yet, or move_group
crashed during startup (check `/tmp/full_stack.log`).

---

## Why these scripts exist (the gotchas they encode)

* **No duplicate RViz.** Every launcher invokes `kill_ros.sh` before
  spawning a new stack, so you can't accidentally end up with two
  RViz windows and two `controller_manager`s fighting for ports.
* **`pkill` self-kill avoidance.** Patterns are stored in `kill_ros.sh`
  itself rather than passed on the command line — that keeps them out
  of the caller shell's argv, so `pkill -f "ros2 launch"` doesn't
  match the running script and SIGKILL the parent.
* **Fail-fast on real-hw network.** `launch_real.sh` pings the cabinet
  and TCP-probes Dashboard (29999) and RTDE (30004) before invoking
  `ros2 launch`. Bad cable / wrong subnet → clear error in under 3 s,
  instead of a 30-second timeout deep inside the driver.
* **Logs survive.** Stack launches write to `/tmp/full_stack.log`; tail
  it any time with `tail -f /tmp/full_stack.log`.

---

## What these scripts are NOT

* They don't `colcon build`. Build once after you change C++/xacro;
  re-running these scripts picks up the new install/ overlay
  automatically.
* They don't touch the pendant. Pendant prep (URP load, Remote Control,
  Tool I/O ownership) is still manual; see the comments at the top of
  `launch_real.sh` for the full checklist.
* They don't manage the gripper outside `play_pickplace.py`. Standalone
  binary close/open testing lives in `tests/onrobot_io_grip.py`.

---

## Quick reference for `play_pickplace.py` flags

| Flag | Effect |
|---|---|
| (none) | All 20 cycles, sim gripper (RViz only) |
| `--max N` | Stop after N pick-place steps (so `--max 4` ≈ 2 full cycles) |
| `--real-gripper` | Drive the real RG6 via UR Tool I/O (binary close/open) |
| `--force F` | IGNORED in `--real-gripper` mode (binary I/O can't set force) |

For the full arg list:

```bash
bash scripts/play_pickplace.sh --help
```

# Hardware test handoff — 2026-05-28

Self-contained checklist for tomorrow's real-hardware run. Whoever picks
this up can work through it top-to-bottom without rereading the chat
history. Each step has an explicit acceptance criterion and a "if this
breaks" pointer.

**Goal of the session:** prove the OnRobot RG6 closes on a wood block
under ROS 2 control via UR Tool I/O, then run a full 1-cycle pickplace
on real hardware, then scale to 20 cycles.

**Estimated wall time:** 90–120 min, assuming the cell is unchanged
from 2026-05-27 evening.

---

## Prereqs — read before walking to the cell

| Item | How to confirm | If missing |
|---|---|---|
| UR10e cabinet powered, RUNNING / NORMAL | Pendant top-left | Power on, init, brake-release |
| Laptop NIC at `192.168.1.35` on the cell subnet | `ip addr show eth0` (WSL) — should list .35 | See [`docs/WSL2_UR10e_NETWORKING.md`](docs/WSL2_UR10e_NETWORKING.md) |
| Cabinet reachable | `ping -c 2 192.168.1.100` returns OK | Bad cable / wrong subnet / cabinet network config |
| Windows Firewall inbound 50001–50004 open | One-time setup, see [`SESSION_HANDOFF.md`](SESSION_HANDOFF.md) 2026-05-26 evening checkpoint | Re-add rule per that checkpoint |
| External Control URCap installed on cabinet | Pendant → URCaps shows "External Control" | One-time install per UR docs |
| OnRobot URCap installed on cabinet | Pendant → URCaps shows "OnRobot" | One-time install |
| Two wood blocks placed at the pick footprint | Place by eye where the RG6 fingers approach | — |

---

## Step 0 — pendant prep (one-time today, but verify)

**0a. Tool I/O ownership.** This is the new one for tomorrow.

```
Pendant → Installation → General → Tool I/O → "Controlled by: User"
```

This stops the OnRobot URCap from re-asserting tool digital out 0
between our `set_io` writes. Without this, our writes get clobbered.

**Acceptance:** the field reads "User" (not "OnRobot RG").

**0b. URP loaded and Remote Control mode.**

Pendant → File → Open `external_control.urp` (the version WITHOUT the
OnRobot RG program node above External Control — the simpler URP we used
2026-05-26).

Top-right corner: switch to **Remote Control** mode.

Press **Play** on the URP. The pendant log should show "Connected to:
Reverse Interface" within a few seconds.

**Acceptance:** URP is PLAYING with the External Control socket waiting
on port 30002. The robot is not moving but is "live".

---

## Step 1 — bring up the ROS 2 stack against the cabinet

From a fresh WSL Ubuntu-22.04 shell:

```bash
cd ~/ur_rg6_ws
bash scripts/launch_real.sh
```

What this does (read the script header if curious):
1. Kills any leftover ROS processes — no duplicate RViz.
2. Pings the cabinet at 192.168.1.100; fast-fails if down.
3. TCP-probes Dashboard (29999) and RTDE (30004); fast-fails if down.
4. Sources `/opt/ros/humble/setup.bash` + workspace overlay.
5. `ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
   use_fake_hardware:=false robot_ip:=192.168.1.100`.
6. Waits up to 30 s for `move_group` to come up, then prints PIDs +
   log tail.

Override the IP if the cabinet's been moved: `bash scripts/launch_real.sh 192.168.1.123`.

**Acceptance:** the script prints "move_group up after Ns" and the
following PIDs are alive:
- `ros2_control_node` (or `ur_ros2_control_node`)
- `robot_state_publisher`
- `move_group`
- An RViz window opens showing the URDF model matching the cabinet's pose.

**If this breaks:** `tail -200 /tmp/full_stack.log`. The most common
failure mode is the driver hanging on RTDE; rerun `scripts/launch_real.sh`
(it will kill the leftover first). If RViz shows the robot in a wildly
different pose than the pendant, the URDF kinematic fix has been
reverted — check `git log src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro`.

---

## Step 2 — verify Tool I/O write path WITHOUT moving the arm

This is the critical new test. **Before** running the pickplace, prove
the `set_io` → tool digital out 0 → RG6 wiring is intact.

```bash
# In a SECOND WSL shell (the first is now busy running the stack):
cd ~/ur_rg6_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Inspect what IOStates actually publishes — we need this to find the
# tool DI pin for grip-detect (currently TODO in onrobot_io_grip.py).
ros2 topic echo /io_and_status_controller/io_states --once | head -40
```

Note where you see the tool digital outs (likely `digital_out_states`
indices 16-17) and tool digital inputs (likely `digital_in_states`
indices 16-17). Record actual pin numbers in
[`wiki/known_bugs_and_workarounds.md`](wiki/known_bugs_and_workarounds.md).

Then exercise the helper:

```bash
python3 tests/onrobot_io_grip.py cycle
# Expected: console prints "CLOSE: ok=True", short pause, "OPEN: ok=True"
# Expected: gripper physically closes ~25 mm width, pauses ~1.5 s, opens ~80 mm
```

**Acceptance criteria (all three must hold):**
1. Console reports `ok=True` for both close and open.
2. RG6 fingers actually move (close → open).
3. `set_io` calls don't fight the URCap (no oscillation, no warning
   messages on the pendant).

**If gripper doesn't move:**
- Recheck Step 0a (Tool I/O Controlled by: User).
- Try `ros2 service list | grep set_io` — should show `/io_and_status_controller/set_io`.
- Try `ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1.0}"` directly. If THIS works but `cycle` doesn't, the helper's bug. If neither works, the cabinet's Tool I/O config is still wrong.

**If gripper oscillates:** the OnRobot URCap is still controlling pin 16 → Tool I/O ownership is still wrong → recheck Step 0a.

---

## Step 3 — single-cycle pickplace WITHOUT real gripper (motion sanity)

Before grabbing real blocks, prove the arm still hits the right
positions with no calibration drift overnight:

```bash
bash scripts/play_pickplace.sh --max 4
# = 2 full pick+place cycles, sim gripper (no real RG6 actuation)
```

Watch RViz **and** the physical arm:
- Approach poses should be ~10 cm above the wood blocks.
- Descent should bring the gripper fingers around the block (NOT into it).
- Pick at WP_2 (top block of pass 1) should align with the actual block X/Y.

**Acceptance:** the arm reaches all 4 waypoint positions without
collision or pendant emergency stop. Logged TCP errors stay <5 mm at
each waypoint (visual estimate is fine — exact reading via
`ros2 topic echo /tf` if needed).

**If alignment is off:** WAYPOINT_TOOL_CALIBRATION_M may need
re-tuning. Current value is `(-0.00666, +0.01052, +0.045)`. See
SESSION_HANDOFF 2026-05-26 evening for the derivation. **Don't proceed to Step 4 if alignment is bad** — gripping with wrong X/Y crashes the block into the table or the gripper finger into the foamboard.

---

## Step 4 — single-cycle pickplace WITH real gripper

The moment of truth:

```bash
bash scripts/play_pickplace.sh --max 4 --real-gripper
```

The `--real-gripper` flag routes each `grip(width)` call through
[`tests/onrobot_io_grip.py`](tests/onrobot_io_grip.py):
- `width < 60 mm` → CLOSE (pin 16 HIGH), 1.5 s settle.
- `width ≥ 60 mm` → OPEN (pin 16 LOW), short settle.

**Acceptance (all four):**
1. Arm reaches WP_2 (pick approach).
2. Descends to grip position. Gripper closes around the block.
3. Lifts (no collision with lower stack).
4. Moves to WP_3 (place). Opens. Block stays on the destination
   pedestal.

**If grip fails to engage:** Step 2 verified the wiring, so a grip
failure here means either timing (block moved before grip closed) or
the close-blocking 1.5 s wasn't enough. Bump
`GRIP_DETECT_TIMEOUT_S` in `tests/onrobot_io_grip.py` to 2.0 s and
rerun.

**If grip engages but block slips during lift:** the RG6 default
force is too low for the wood block weight. Binary Tool I/O can't
adjust force — flag for the future Compute Box or RS-485 URCap
upgrade (see `wiki/decisions.md` 2026-05-27 entry).

**If block placement is off:** the +45 mm Z calibration was tuned at
gripper-down orientation. At PLACE orientation the wrist rotation is
different → tool-frame error manifests differently in world frame.
If the place Z is off by >1 cm, document the new value and we may
need a separate PLACE_TOOL_CALIBRATION_M.

---

## Step 5 — full 20-cycle run

If Step 4 passes, scale up:

```bash
bash scripts/play_pickplace.sh
# = all 20 cycles (10 picks from top stack, 10 picks from bottom stack)
# ~50 s/cycle observed → ~17 min wall time
```

**Acceptance:** all 20 cycles complete without intervention. No
collisions. Final block stack is visually correct.

**If a single cycle fails:** the script aborts. Re-run with `--max
<failed_cycle_step>` to isolate; check whether the failure is
calibration drift (Z slightly different across the stack) or
grip-slip.

---

## Step 6 — tear down

```bash
bash scripts/kill_ros.sh
```

On the pendant: stop the URP. Cabinet stays powered for next session
unless leaving the cell.

---

## After the run — durable findings

If anything non-obvious came up (new pin numbers, calibration changes,
grip-force limit hit, etc.), write it to the wiki BEFORE closing the
session:

- New pin numbers / IOStates structure → `wiki/known_bugs_and_workarounds.md`
- Calibration changes → `tests/play_pickplace.py` + comment why +
  update `wiki/decisions.md` if the change reflects a design choice
- Grip-detect implementation (replace fixed timeout with real DI edge) →
  `tests/onrobot_io_grip.py` `close_blocking()` + delete the TODO comment

Then update `SESSION_HANDOFF.md` with a new checkpoint reflecting what
was verified, and `git push`.

---

## Quick reference

| Need | Command |
|---|---|
| Bring up stack (real) | `bash scripts/launch_real.sh` |
| Bring up stack (sim) | `bash scripts/launch_sim.sh` |
| Tear down | `bash scripts/kill_ros.sh` |
| Pickplace, motion only | `bash scripts/play_pickplace.sh --max 4` |
| Pickplace, real gripper | `bash scripts/play_pickplace.sh --max 4 --real-gripper` |
| Gripper standalone | `python3 tests/onrobot_io_grip.py cycle` |
| Inspect I/O states | `ros2 topic echo /io_and_status_controller/io_states --once` |
| Tail launch log | `tail -f /tmp/full_stack.log` |

---

## Failure modes and quick references

| Symptom | First check |
|---|---|
| `scripts/launch_real.sh` says "cabinet not reachable" | Laptop NIC config, cable, cabinet power |
| `scripts/play_pickplace.sh` says "move_group is NOT running" | Step 1 didn't finish — `tail -50 /tmp/full_stack.log` |
| Two RViz windows | Step 1's kill phase didn't catch a leftover — `bash scripts/kill_ros.sh` then retry |
| Gripper doesn't move at all | Step 0a (Tool I/O ownership) |
| Gripper closes but opens immediately | URCap still controlling pin 16 — Step 0a |
| Arm moves but lands wrong position | Calibration drift or URDF revert — see Step 3 |
| Pendant safety stop | Don't restart — check the pendant log, then `bash scripts/kill_ros.sh` and start over |

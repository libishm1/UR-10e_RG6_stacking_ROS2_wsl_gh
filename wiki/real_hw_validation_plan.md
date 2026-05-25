# Real-hardware validation plan

## Purpose

A single-page, step-by-step plan for taking this workspace from the
fake-hardware baseline (`use_fake_hardware:=true`, 10/10 sim runs) to
verified motion on the physical UR10e + RG6 cell at
`192.168.1.100`. Each step has a **pass criterion**, an **abort
criterion**, and **what to record in the validation log** below.

The whole plan is designed so a single human at the cell can run it
end-to-end with a hand on the E-stop. Total estimated wall time: 30–60
minutes once the pendant prereqs are done.

**Hard rule:** do not advance to a later step if an earlier step
fails. Diagnose first.

---

## Phase 0 — Pre-flight (off the robot, off the network)

| # | Action | Pass criterion | Where |
|---|---|---|---|
| 0.1 | Workspace built and sourced (`source ~/ur_rg6_ws/install/setup.bash`) | `ros2 pkg list \| grep ur10e_rg6_moveit_config` returns the package | WSL |
| 0.2 | `~/.wslconfig` has `networkingMode=mirrored` | `cat /mnt/c/Users/libish\ m/.wslconfig` shows it | Windows |
| 0.3 | Windows firewall allows inbound TCP 50001-50002 (one-time) | `Get-NetFirewallRule -DisplayName "UR External Control 50001-50002"` exists | PowerShell admin |
| 0.4 | `Set-NetFirewallHyperVVMSetting` override applied (one-time) | per [docs/WSL2_UR10e_NETWORKING.md](../docs/WSL2_UR10e_NETWORKING.md) Level 1 | PowerShell admin |
| 0.5 | All current sim stacks killed | `pgrep -af ros2_control_node` returns empty | WSL |

**Abort:** any FAIL at this phase → consult `docs/WSL2_UR10e_NETWORKING.md`.

---

## Phase 1 — Pendant prereqs (at the cell, no ROS yet)

These are one-time per cabinet (resets after factory reset). Skip if
already done previously and verified.

| # | Action | Pass criterion | Notes |
|---|---|---|---|
| 1.1 | Pendant → Settings → Security → enable **all 5 services** (29999/30001/30002/30003/30004) | Toggles flip from blocked to allowed | Without this, every TCP probe times out. |
| 1.2 | Pendant → Settings → Security → General → change "Disable inbound access to additional interfaces (by port)" to `1-21,23-65535` | Port 22 excluded from blocklist | Lets SSH work (for Path B fallback later). |
| 1.3 | Pendant → Settings → Security → Secure Shell → enable, auth "Both" | SSH enabled | Robots-workspace key already imported per `D:\robot_ws\robots\outputs\2026-05-09\ssh_setup\`. |
| 1.4 | Pendant → top-right toggle → **Remote Control mode** | Mode indicator shows Remote | Required for ROS-driven trajectory streaming. |
| 1.5 | External Control URCap installed (UR vendor) | Visible in Settings → System → URCaps | If missing: install from https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases |
| 1.6 | OnRobot URCap installed and gripper recognised | URCap UI shows "RG[0]: 33" or similar | Already on this pendant. |
| 1.7 | Create `external_control.urp` program: drag External Control URCap into program tree, set **Host IP = `192.168.1.35`** (WSL host IP) and **Custom Port = `50002`**. Save. | Program saved, can be opened | Do NOT press Play yet. |
| 1.8 | Network confirmed: pendant Settings → System → Network shows `192.168.1.100/24`, no gateway, "Network is connected: GREEN" | Green indicator | If red: re-run "Apply" cycle. |
| 1.9 | **OnRobot URCap cold-boot workaround:** cold-boot the cabinet, immediately restart it (without Play) | Cabinet boots twice cleanly | Documented in [`real_hw_connection.md`](real_hw_connection.md). |

**Abort:** any FAIL at this phase → defer real-hw validation. The cell
is not ready.

---

## Phase 2 — Network reachability (WSL ↔ cabinet)

Run from a WSL terminal (no ROS launch yet).

| # | Action | Pass criterion |
|---|---|---|
| 2.1 | `~/ur_rg6_ws/tests/check_real_hw_network.sh 192.168.1.100` | Script exits 0, all 5 TCP probes pass, Dashboard handshake reports `remote control: true` and `safetystatus: NORMAL` |
| 2.2 | Manual: `ping -c 3 192.168.1.100` | < 5 ms round-trip |
| 2.3 | Manual: `nc -vz 192.168.1.100 30004` | succeeded |

**Abort:** any FAIL → see [`WSL2_UR10e_NETWORKING.md`](../docs/WSL2_UR10e_NETWORKING.md)
"Diagnostic recipes". Do not advance to Phase 3.

---

## Phase 3 — Driver bring-up (handshake only, no motion)

| # | Action | Pass criterion |
|---|---|---|
| 3.1 | `ros2 launch ur10e_rg6_moveit_config full_stack.launch.py use_fake_hardware:=false robot_ip:=192.168.1.100` | Stack reaches `[move_group-…] You can start planning now!` within ~20 s, no FATAL errors |
| 3.2 | At the pendant: **open `external_control.urp` and press Play** | Driver logs `Robot connected to reverse interface` + `Ready to receive control commands` |
| 3.3 | `ros2 control list_controllers` shows `scaled_joint_trajectory_controller` ACTIVE | active row |
| 3.4 | `ros2 topic hz /joint_states` reports steady 500 Hz | mean ≥ 480 Hz, no dropouts |
| 3.5 | Single RViz window opens with the robot at its boot pose (live, not ghost) | Single RViz process, robot visible |

**Abort:** any FAIL at this phase → `pkill -9 -f ros2_control_node` and
restart the launch. If "connection refused on 50001/50002" persists,
see [`real_hw_connection.md`](real_hw_connection.md) → "Diagnostic
recipes" → "Reverse channel won't connect".

---

## Phase 4 — Read-only kinematic verification (zero motion)

Verifies the URDF kinematic model agrees with the physical robot. No
trajectories sent. Robot stays exactly where Phase 3 left it.

| # | Action | Pass criterion |
|---|---|---|
| 4.1 | `pip3 install --user ur_rtde` (one-time) | Install completes |
| 4.2 | `python3 ~/ur_rg6_ws/tests/measure_real_robot_pose.py --host 192.168.1.100` while the robot is at HOME | Script exits 0. Joint Δ from HOME_Q ≤ 1° on all 6 joints. TCP yaw delta vs ROS HOME ≤ 2°. |
| 4.3 | Visually compare RViz robot orientation to the real cabinet at HOME | Both show same arm direction (cable side, gripper hanging direction) OR record the visible delta yaw |

**Pass criterion 4.3 — if mismatch:** measure the yaw angle (in
degrees) between the real cabinet's "manufacturer front" and our
URDF's `+X` direction. That's the value to put in
`<origin rpy="0 0 RADIANS">` of the URDF ur_robot mount. Document in
the validation log.

**Abort:** if joint values differ from HOME_Q by > 1° → robot is not at
HOME, or our HOME constant is wrong. Do not advance.

---

## Phase 5 — Arm-only smoke test (small motion, slow)

The first actual motion. 5% speed cap, ±0.05 rad shoulder_lift nudge
(≈ ±3 cm TCP), hard-capped at ±0.10 rad regardless of CLI args.

| # | Action | Pass criterion |
|---|---|---|
| 5.1 | `python3 ~/ur_rg6_ws/tests/real_hw_smoke.py` (DRY RUN — no `--yes`) | Prints HOME/UP/HOME/DOWN/HOME joint vectors. NO motion. |
| 5.2 | Hand on E-stop. People out of work envelope. Confirm pendant is in Remote Control mode. | Operator ready |
| 5.3 | `python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --no-gripper` | Arm moves through HOME → UP → HOME → DOWN → HOME at 5% speed. Each move completes without `CONTROL_FAILED`. Returns to HOME at the end. |
| 5.4 | Observe: TCP motion in the ±Z direction matches the dry-run prediction (≈ 3 cm vertical) | Visual confirmation |

**Abort:** any `CONTROL_FAILED` that does NOT auto-retry-as-PTP →
press E-stop, kill the script, diagnose. Do not advance.

---

## Phase 6 — Gripper-only smoke (gripper actuates, no arm motion)

Verifies the URCap path through `/urscript_interface/script_command`.
This is the FIRST URCap call — watch for the cold-boot quirk.

| # | Action | Pass criterion |
|---|---|---|
| 6.1 | `python3 ~/ur_rg6_ws/tests/gripper_test.py --no-arm --real --force 25 --widths 100 80 100 --hold 2.0` | Gripper closes from 100 mm to 80 mm and back. Pendant Log shows OnRobot URCap activity. |
| 6.2 | If cold-boot quirk fires (`RG grip didn't initialize`): cold-boot the cabinet, immediately restart, retry | Second attempt succeeds |
| 6.3 | Observe fingers physically open and close | Visual confirmation, no fingertip damage |

**Abort:** if gripper never actuates and the URCap cold-boot fix doesn't
help → diagnose URCap config (pendant → URCaps → OnRobot panel).
Do not advance.

---

## Phase 7 — Combined arm + gripper smoke

Arm motion + URCap gripper in the same script. Identical hard caps
(±0.10 rad, 5% speed, 25 N grip force).

| # | Action | Pass criterion |
|---|---|---|
| 7.1 | `python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --real-gripper --force 25` | Full cycle: HOME → open 100 mm → UP → close 80 mm → HOME → DOWN → open 100 mm → HOME → open 100 mm. No errors. |
| 7.2 | Repeat with `--cycles 3` for endurance | 3 cycles complete clean |

**Abort:** any URCap stall mid-script → press E-stop. The
URScript-interface gotcha is that wrapped programs interrupt External
Control; but our `rg_grip(...)` calls are single-line "secondary
programs" and should NOT interrupt. If they do, see
[`real_hw_connection.md`](real_hw_connection.md) → "Known issues".

---

## Phase 8 — One pick-place cycle (real boxes)

Boxes are visualised in RViz but NOT physically present unless you've
placed real objects at the URScript pick poses. For the first run, do
NOT place real objects — verify the trajectory geometry first.

| # | Action | Pass criterion |
|---|---|---|
| 8.1 | `python3 ~/ur_rg6_ws/tests/play_pickplace.py --real-gripper --force 25 --max 1` | Exactly one full pick-place cycle completes: approach → pick (close) → lift → transit → place (open) → retract. No errors. |
| 8.2 | Observe at the cell: gripper reaches the correct world position for WP_2 (pick) and WP_6 (first place) | Visual + measure: TCP within ±5 mm of expected world XYZ |
| 8.3 | After completion, gripper opens to 70 mm (bookend) and arm returns to HOME | Final state matches start state |

**Abort:** TCP off by more than 5 mm → extract proper calibration
(`ros2 launch ur_calibration calibration_correction.launch.py
robot_ip:=192.168.1.100 target_filename:=...`) and rebuild
`ur10e_rg6_moveit_config`. Do not advance.

---

## Phase 9 — Full 10-cycle pick-place (normal force)

Once Phase 8 passes, ramp up.

| # | Action | Pass criterion |
|---|---|---|
| 9.1 | Place real physical objects at the URScript pick positions (table of dimensions known) | Objects within ±5 mm of expected XYZ |
| 9.2 | `python3 ~/ur_rg6_ws/tests/play_pickplace.py --real-gripper --force 40` (full 10 cycles, normal force matching URScript) | All 10 picks succeed, all 10 places succeed, no fall-through, no slip |
| 9.3 | Operator checks final stack heights match URScript design (z = 0.036 / 0.068 / 0.100 / 0.132 m for the place columns) | ±5 mm |

**Abort:** any drop / mis-grip / collision → press E-stop. Diagnose
before re-running.

---

## Validation log (fill in as you go)

```
Date:            ____________
Operator:        ____________
Cell s/n:        20255201551
Polyscope:       5.24.0.1219432

Phase 0: Pre-flight      [ PASS / FAIL ]   notes:
Phase 1: Pendant         [ PASS / FAIL ]   notes:
Phase 2: Network         [ PASS / FAIL ]   notes:
Phase 3: Driver bring-up [ PASS / FAIL ]   notes:
Phase 4: Read-only kine  [ PASS / FAIL ]   joint Δ max = ____°,  TCP yaw Δ = ____°
Phase 5: Arm smoke       [ PASS / FAIL ]   any CONTROL_FAILED?  ____
Phase 6: Gripper smoke   [ PASS / FAIL ]   cold-boot retry needed?  ____
Phase 7: Combined smoke  [ PASS / FAIL ]   endurance cycles:  ____
Phase 8: Single pick-place  [ PASS / FAIL ]   TCP error (mm):  ____
Phase 9: Full 10-cycle      [ PASS / FAIL ]   drops / slips:  ____

URDF mount yaw correction needed:  __________ rad (set in
src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro
  <xacro:ur_robot ...><origin rpy='0 0 YAW'/></xacro:ur_robot>)
```

---

## Rollback plan

If anything goes wrong mid-validation:

1. **E-stop** the pendant immediately.
2. From WSL: `pkill -9 -f 'ros2 launch'` to kill the whole driver stack.
3. At the pendant: press Stop on the External Control program.
4. Cabinet stays in Remote Control mode; no need to power-cycle unless
   the URCap is unresponsive.
5. Restart from Phase 3 once the failure is diagnosed.

If a destructive collision happens: power-cycle the cabinet, inspect
hardware, then start from Phase 0 (full verification).

---

## What this plan deliberately does NOT cover

- **Force-mode control.** Out of scope — pick-place doesn't need it.
- **Continuous teleop.** Use this plan to bring the cell up, then use
  RViz interactive markers or your own teleop node.
- **Multi-machine ROS 2 networking.** Single-machine WSL2 only.
- **CI / regression suite.** No way to run pick-place automatically on
  real hardware — every Phase 9 run requires a human at the cell.

## Related

- [`docs/WSL2_UR10e_NETWORKING.md`](../docs/WSL2_UR10e_NETWORKING.md) —
  fallback ladder if Phase 2 fails
- [`real_hw_connection.md`](real_hw_connection.md) — port + URCap
  details for Phases 3 and 6
- [`launch_files.md`](launch_files.md) — what each `.launch.py` does
- `tests/check_real_hw_network.sh` — Phase 2 automation
- `tests/measure_real_robot_pose.py` — Phase 4 automation
- `tests/real_hw_smoke.py` — Phases 5 and 7 automation
- `tests/play_pickplace.py` — Phases 8 and 9 automation

## Last updated

2026-05-25.

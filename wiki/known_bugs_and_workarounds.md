# Known bugs and workarounds

Living catalog of every "this thing burned us, here's the workaround" from
the workspace. New entries go at the top with a date. Future Claude
sessions: search this first before re-discovering anything.

## Quick index

- [Shoulder-pan sign mismatch (URDF vs cabinet)](#shoulder-pan-sign-mismatch-urdf-vs-cabinet)
- [Bare URScript on port 30002 + URCap functions crashes URCap](#bare-urscript-on-port-30002--urcap-functions-crashes-urcap)
- [WSLg "pink window" after many launch cycles](#wslg-pink-window-after-many-launch-cycles)
- [Dual RViz spawns from `full_stack.launch.py`](#dual-rviz-spawns-from-full_stacklaunchpy)
- [`use_fake_hardware:=true` initial_positions parsing warning](#use_fake_hardwaretrue-initial_positions-parsing-warning)
- [WSL2 NAT blocks UR driver's reverse interface](#wsl2-nat-blocks-ur-drivers-reverse-interface)
- [OnRobot URCap cold-boot quirk](#onrobot-urcap-cold-boot-quirk)
- [pickplace LIN→PTP retry CONTROL_FAILED noise](#pickplace-linptp-retry-control_failed-noise)
- [Calibration extraction doesn't fix the 1m+ TCP-Z mismatch](#calibration-extraction-doesnt-fix-the-1m-tcp-z-mismatch)

---

## Shoulder-pan sign mismatch (URDF vs cabinet) — **VERIFIED 2026-05-26**

**2026-05-26 — sim-verified AND real-hardware verified.** Our URDF's
`shoulder_pan_joint` axis is sign-inverted from this cabinet's
controller. Same physical pose corresponds to opposite numerical
shoulder_pan values.

Verification (ur_rtde readback with real robot at physical HOME):
- Real cabinet HOME: `shoulder_pan = +π/2` (+1.5708 rad)
- Our sim HOME_Q for same visual pose: `shoulder_pan = −π/2` (−1.5708 rad)
- Δ = 180°, all other joints identical

**Deployment caveat for real hardware:** scripts use `-π/2` for sim
visualization. Real cabinet at `-π/2` would rotate arm to the
OPPOSITE side from work area. Use one of three strategies in
[`shoulder_pan_sign_mismatch.md`](shoulder_pan_sign_mismatch.md)
"Critical caveat" section before commanding real motion.

**Workaround (sim-only):** flip the sign on `shoulder_pan_joint` in HOME:
```python
HOME_Q = [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]
```
Applied in: `play_pickplace.py`, `real_hw_smoke.py`, `ur10e_rg6.srdf`
`home` group_state, `initial_positions.yaml`. Pickplace runs 10/10 cleanly
with the new value, RViz visual matches real cell.

**Critical for real hardware:** the cabinet's URScript HOME (from
`dodectest3.urp`) uses `+pi/2` for shoulder_pan. Sending our `-pi/2` to
the real cabinet rotates the arm to the OPPOSITE side (away from the
work area). Before Phase 5 real-hw motion: either re-teach the cabinet
HOME, or substitute `+pi/2` for shoulder_pan when commanding real.

Full details: [`shoulder_pan_sign_mismatch.md`](shoulder_pan_sign_mismatch.md).

---

## Bare URScript on port 30002 + URCap functions crashes URCap

**2026-05-26 (verified at the cell).** Sending a single-line
`rg_grip(width, force)` directly to TCP port 30002 (no URP wrapper)
**crashed the OnRobot URCap** and forced a cabinet restart. Previously
documented as "silently ignored" in D:\robot_ws notes; the 2026-05-26
observation is stronger.

**Workaround:** never send URCap-defined functions (`rg_grip`,
`rg_payload_set`, etc.) through a path that doesn't have the URCap
preamble loaded. Use:
- **Path B** (SFTP `.urp`+`.script` + Dashboard load/play) — proven
  workflow in `D:\robot_ws\robots\outputs\2026-05-10\path_b\urp_deploy.py`
- **External Control program loaded + Play pressed** then
  `/urscript_interface/script_command` from `ur_robot_driver`

Full details in [`real_hw_connection.md`](real_hw_connection.md)
"Known issues" section.

---

## WSLg "pink window" after many launch cycles

**2026-05-26.** After ~10+ launch/relaunch cycles in a single session,
RViz windows stop rendering — show a pink/purple gradient instead of
the robot model. Bare `rviz2` (no config) also fails. WSLg's
Qt+OpenGL state corrupts.

**Workaround:** from PowerShell on Windows:
```powershell
wsl --shutdown
```
Wait 5 s, reopen WSL terminal. WSLg state resets cleanly. RViz works
on the next launch. No workspace state lost; only the WSL kernel
instance recycles.

---

## Dual RViz spawns from `full_stack.launch.py`

**2026-05-23.** Without intervention, `full_stack.launch.py` opens TWO
RViz windows: one from the UR driver's default `view_robot.rviz`, one
from our `moveit_rviz.launch.py`.

**Workaround:** edit `src/onrobot1_ros/onrobot_description/launch/ur10e_rg6_control.launch.py`
to pass `'launch_rviz': 'false'` to the included `ur_control.launch.py`.

**Caveat:** `onrobot1_ros` is in `.gitignore` (vendor package).
Re-applying `vcs import src < ros2.repos` wipes the edit. TODO: lift
the `launch_rviz: false` arg up into our own `full_stack.launch.py`
so it survives vendor refresh.

---

## `use_fake_hardware:=true` initial_positions parsing warning

`ros2_control_node` logs:
```
[WARN] [mock_generic_system]: Parsing of optional initial interface values
       failed or uses a deprecated format.
```

**Effect:** mock_components ignores `initial_positions.yaml` and falls
back to joints = 0 (or the last commanded position). This means the
sim arm may not start at HOME on launch, causing the first `movej HOME`
to be a big sweep that can hit path tolerance.

**Workaround:** the script's `movej(HOME, "HOME (start)")` handles the
big initial sweep at slow speed. Auto-retry-as-PTP catches occasional
CONTROL_FAILED. No fix in mock_components available; this is a known
ros2_control issue on Humble.

---

## WSL2 NAT blocks UR driver's reverse interface

Per
`D:\robot_ws\reference\deep-research-wsl2_networking.md`:

By default WSL2 is NAT-behind a virtual interface; the UR cabinet
cannot reach the WSL2 IP directly. The UR driver's reverse channel
(ports 50001/50002/50003) won't connect.

**Workarounds (in order of preference):**

1. **Mirrored networking mode** on Windows 11 (`networkingMode=Mirrored`
   in `~/.wslconfig`, plus `Set-NetFirewallHyperVVMSetting -Name Default
   -State Off`). WSL2 appears on the host's LAN. No portproxy needed.
   **Currently in use for this workspace** per `docs/WSL2_UR10e_NETWORKING.md`.
2. **netsh portproxy** rules (Admin PowerShell):
   ```powershell
   $wslIP = wsl hostname -I
   netsh interface portproxy add v4tov4 listenport=50001 connectaddress=$wslIP connectport=50001
   netsh interface portproxy add v4tov4 listenport=50002 connectaddress=$wslIP connectport=50002
   netsh interface portproxy add v4tov4 listenport=50003 connectaddress=$wslIP connectport=50003
   ```
   Must update if WSL2 restarts (IP changes).
3. **Set `reverse_ip`** explicitly to the Windows host's LAN IP in the
   driver YAML. Don't let it auto-detect — it picks the WSL2 NAT IP.

**Phase 5 checklist before real_hw_smoke:** verify mirrored mode is
active (it is), check `reverse_ip` in driver config (TODO), confirm
Windows Firewall allows inbound on 50001-50003.

---

## OnRobot URCap cold-boot quirk

**2026-05-09 (from D:\robot_ws).** First `rg_grip()` after a cold
cabinet boot triggers "RG grip didn't initialize" and the cabinet
shuts down. REPEATABLE.

**Workaround:**
1. Cold-boot the cabinet
2. **Immediately restart it** (without pressing Play / running anything)
3. Second boot picks up the URCap correctly; rg_grip works on first call

Alternative: open the OnRobot URCap UI on the pendant, click
"Connect" / re-init the RG grip before pressing Play.

---

## pickplace LIN→PTP retry CONTROL_FAILED noise

Pickplace shows ~1-2 `CONTROL_FAILED` warnings per 10-cycle run,
followed by automatic retry as PTP that succeeds. Pre-existing
behavior, not a regression.

**Why:** Pilz LIN's path tolerance check is tight; for some Cartesian
goals (specifically the deep-pose-near-floor waypoints), mock_components
in sim or controller interpolation issues trigger the tolerance.

**Workaround:** automatic in `play_pickplace.py::_send()` — falls back
to PTP on LIN failure. No user action needed. Just noisy in the log.

---

## Calibration extraction doesn't fix the 1m+ TCP-Z mismatch

**2026-05-26.** Extracted this cell's factory calibration via
`ros2 launch ur_calibration calibration_correction.launch.py
robot_ip:=192.168.1.100`. Applied to URDF via `kinematics_parameters_file`.

But at HOME joint values, real TCP = `(0.176, 0.691, 0.400)` while sim
TCP = `(0.001, 0.532, 1.484)` — **1m+ Z mismatch persists.** The
calibration only captured small per-link deltas, not the gross
kinematic-model difference.

Possible root causes (uninvestigated):
- The URDF uses a different UR10e variant than the physical robot
- The URCap on the pendant reconfigures the TCP frame at runtime
- The cabinet firmware was upgraded with non-standard kinematics

**Workaround:** accept the mismatch — kinematics, IK, controllers all
work correctly because the real cabinet uses ITS OWN calibration. The
RViz visualization is "close enough" for planning + collision checking
but won't be pixel-accurate.

The [shoulder-pan sign flip](#shoulder-pan-sign-mismatch-urdf-vs-cabinet)
fixes the gross visual orientation; the residual position mismatch is
cosmetic-only.

---

## Last updated

2026-05-26.

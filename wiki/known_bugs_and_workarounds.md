# Known bugs and workarounds

Living catalog of every "this thing burned us, here's the workaround" from
the workspace. New entries go at the top with a date. Future Claude
sessions: search this first before re-discovering anything.

## Quick index

- [Pendant vs RTDE 400 mm Z gap on this cabinet (cosmetic)](#pendant-vs-rtde-400-mm-z-gap-on-this-cabinet-cosmetic)
- [Combined URDF must forward `script_filename` & friends for real hardware](#combined-urdf-must-forward-script_filename--friends-for-real-hardware)
- [`onrobot_interface` C++ plugin crashes on init — never use it](#onrobot_interface-c-plugin-crashes-on-init--never-use-it)
- [RTDE "Pipeline producer overflowed" spam on WSL2](#rtde-pipeline-producer-overflowed-spam-on-wsl2)
- [`ros2 topic` CLI hangs in WSL2; rclpy direct subscriber works](#ros2-topic-cli-hangs-in-wsl2-rclpy-direct-subscriber-works)
- [Shoulder-pan sign mismatch (URDF vs cabinet)](#shoulder-pan-sign-mismatch-urdf-vs-cabinet)
- [Bare URScript on port 30002 + URCap functions crashes URCap](#bare-urscript-on-port-30002--urcap-functions-crashes-urcap)
- [WSLg "pink window" after many launch cycles](#wslg-pink-window-after-many-launch-cycles)
- [Dual RViz spawns from `full_stack.launch.py`](#dual-rviz-spawns-from-full_stacklaunchpy)
- [`use_fake_hardware:=true` initial_positions parsing warning](#use_fake_hardwaretrue-initial_positions-parsing-warning)
- [WSL2 NAT blocks UR driver's reverse interface](#wsl2-nat-blocks-ur-drivers-reverse-interface)
- [OnRobot URCap cold-boot quirk](#onrobot-urcap-cold-boot-quirk)
- [pickplace LIN→PTP retry CONTROL_FAILED noise](#pickplace-linptp-retry-control_failed-noise)
- [Calibration extraction doesn't fix the 1m+ TCP-Z mismatch](#calibration-extraction-doesnt-fix-the-1m-tcp-z-mismatch)
- [RG6 over RS485/Modbus — operational gotchas](#rg6-over-rs485modbus--operational-gotchas-tool-comm-bridge)

---

## RG6 over RS485/Modbus — operational gotchas (tool-comm bridge)

**2026-05-28.** Driving the RG6 Modbus over the tool flange works, but several
operational traps bite. Full setup + register map: [rg6_rs485_modbus.md](rg6_rs485_modbus.md).

**Prereqs for ANY gripper comms (all must hold):**
- `ros` installation: Tool I/O Controlled by **User** + **Communication Interface**
  (1M/Even/One) + Tool Output Voltage **24** + OnRobot Setup device = **None**.
- rs485 daemon URCap installed on the cabinet (`/root/.urcaps/rs485-1.0.jar`).
- Cabinet firewall: inbound port **54321** allowed (Settings → Security → General).
- `socat` installed on the host (`sudo apt install socat`).
- `external_control.urp` **PLAYING** — the rs485 bridge runs inside the control
  program; without it, 54321 listens but no tool data flows.

**Trap 1 — socat pty locks after ONE open.** The driver's socat uses
`waitslave`; once a client opens `/tmp/ttyUR` and closes it, socat locks the
pty (to the cabinet's 1M baud) and the NEXT open fails `(22) Invalid argument`.
Open ONCE and hold the handle for the whole session (`OnRobotModbusGrip.connect()`
does). For repeated standalone tests, reset socat between runs (Trap 3) or relaunch.

**Trap 2 — rs485 daemon STUCK on a stale TCP connection.** After `wsl --shutdown`,
an **admin-mode WSL** session, or any abrupt host kill, an old socat connection
to 54321 is left half-open (`FIN-WAIT-2`); the cabinet's rs485 daemon keeps
bridging the dead socket and the gripper never replies (Modbus reads → None)
EVEN THOUGH 54321 is open and a new socat is ESTABLISHED. **Replaying External
Control does NOT reset the daemon** (separate URCap service). **Fix: restart
PolyScope (or power-cycle the cabinet).** And **avoid admin-mode WSL** — it
caused the connection churn.

**Trap 3 — fast socat reset (skip a 40 s full relaunch).** Kill the driver's
socat by PID (scan `/proc` for a cmdline that *starts with* `socat` and
contains `ttyUR` — NOT `pkill -f`, which self-matches), then restart it:
`socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:192.168.1.100:54321 &`.
~5 s vs ~40 s.

**Trap 4 — kill_ros.sh self-kill via "dashboard_client".** Don't run a launch
script (it calls `kill_ros.sh`) in the same shell as a command containing a
kill_ros pattern (e.g. `ros2 service call /dashboard_client/...`). kill_ros's
`pkill -f dashboard_client` matches your shell's argv → SIGKILL (exit 9). Keep
launches separate from dashboard commands.

**Calibration / status (2026-05-28):** cmd 160 → ~150 mm (mech max); cmd 0 →
fully closed; `open()`=160 / `close_blocking()`=0 are the demo values. reg 267
(@258 off 9) is width but NON-LINEAR vs fingertip gap — not exact mm.

**grip-detect — use the WIDTH, not the status word (VERIFIED 2026-05-28).**
Controlled test with the block actually REMOVED for the empty close (not just
slid aside): close-on-block → jaws stop at the object, width **~59.9 mm**;
close-on-empty → jaws fully close, **~10.3 mm**. The status word @258 off10 was
the **same (=2) in BOTH** (and read 1 in other runs) → it is NOT a grip flag.
An earlier "off10 bit0 = grip" claim was an artifact of the block being present
in BOTH closes. So **grip-detect = jaws stopped SHORT of the commanded width by
more than ~15 mm** (`GRIP_STOP_MARGIN_MM`): after `close_blocking()` (cmd 0) a
final width >~25 mm means an object is held; ~10 mm means empty.
`onrobot_modbus_grip.py` `grip_detected()`/`grip_to()` now use this width
method. (A duplicate `BIT_GRIP_DETECT` line that silently forced the old bit
check to bit1 was also removed.)

---

## Pendant vs RTDE 400 mm Z gap on this cabinet (cosmetic)

**2026-05-11 (D:\robot_ws), confirmed still present 2026-05-26.** On this
exact cabinet (PolyScope 5.24.0.1219432, S/N 20255201551, OnRobot Quick
Changer + RG6 + current fingers), the **pendant Move screen** and **RTDE
`actual_TCP_pose`** disagree on the Z component of the same TCP by exactly
~400 mm. X and Y agree to <0.05 mm. Magnitude is close to the QC+RG6+finger
stack length.

Documented in detail in `/mnt/d/robot_ws/robots/wiki/ur10e_rg6/tcp_calibration.md`
with 4 tested hypotheses (all rejected) and 4 open diagnostic paths (`get_actual_tcp_pose()`
via URScript, pendant Feature dropdown, etc.).

**Why it doesn't affect ROS 2 motion correctness:** our planning chain is
`URDF tool0 → URDF FK → RTDE TCP → cabinet motion`. The pendant display is
never read by ROS. The URDF FK ↔ RTDE TCP agreement is verified at 0.4 mm
(see `wiki/shoulder_pan_sign_mismatch.md` post-fix verification). The
400 mm pendant gap only matters for tools that read pendant values OR do
their own Rhino-world conversion (Grasshopper's `Plane from UR TCP Pose`
component handled this by injecting `z_offset_mm = -5.605176` which
composites the +394.4 mm Rhino-world offset with the -400 mm gap).

**Workaround for ROS 2:** none needed. Just don't trust the pendant Move
screen's Z for absolute positions on this cabinet — use RTDE or our URDF
FK instead.

---

## Combined URDF must forward `script_filename` & friends for real hardware

**2026-05-26.** First `use_fake_hardware:=false` launch after locking the
URDF crashed `ur_ros2_control_node` with:
```
[UR_Client_Library:]: Opening file 'to_be_filled_by_ur_robot_driver' failed
[resource_manager]: Failed to 'configure' hardware 'ur10e'
... terminate called after throwing 'std::runtime_error'
```
Driver exits with SIGABRT (-6). Cascade: every controller spawner fails
because the controller_manager is dead.

**Root cause.** `ur_macro.xacro` declares default-placeholder values for:
- `script_filename:=to_be_filled_by_ur_robot_driver`
- `output_recipe_filename:=to_be_filled_by_ur_robot_driver`
- `input_recipe_filename:=to_be_filled_by_ur_robot_driver`

Upstream `ur_robot_driver/launch/ur_control.launch.py` passes the real paths
as xacro CLI args when building the URDF. **But** if your top-level xacro
(here `ur10e_rg6.urdf.xacro`) doesn't declare those names as `<xacro:arg>`,
xacro silently drops the CLI args, and the placeholder defaults reach
`URPositionHardwareInterface::on_configure` which tries to open them as
file paths.

**Workaround.** Declare and forward the args in your top-level URDF. The
fix applied to `ur10e_rg6.urdf.xacro`:
1. Add `<xacro:arg name="script_filename" default=""/>` (and equivalents
   for `output_recipe_filename`, `input_recipe_filename`, `reverse_ip`,
   `reverse_port`, `script_sender_port`, `trajectory_port`,
   `script_command_port`, `headless_mode`, `non_blocking_read`,
   `keep_alive_count`).
2. Pass each `script_filename="$(arg script_filename)"` etc. inside the
   `<xacro:ur_robot ...>` invocation, alongside the args already there.

Crib from upstream `ur.urdf.xacro` — it's the canonical list.

**Symptom check.** When debugging, `grep to_be_filled /tmp/full_stack.log`
catches this fast.

---

## `onrobot_interface` C++ plugin crashes on init — never use it

**2026-05-26.** Setting the ros2_control block for `OnRobotRG6System` to
`<plugin>onrobot_interface/OnRobotHardwareInterface</plugin>` (the C++
plugin shipped with `onrobot1_ros/onrobot_interface`) crashes
`ur_ros2_control_node` at startup with the French-locale error:
```
[OnRobotHardwareInterface]: Un seul joint doit être défini. Trouvé : 0
[resource_manager]: Failed to initialize hardware 'OnRobotRG6System'
terminate called: 'Wrong state or command interface configuration.'
missing state interfaces: ' rg6_joint/position '
missing command interfaces: ' rg6_joint/position '
```

The plugin requires `prefix` and `model` `<param>` entries inside the
`<hardware>` block that our URDF doesn't pass, then it reads 0 joints,
then aborts.

**Workaround (LOCKED).** Always use `<plugin>mock_components/GenericSystem</plugin>`
for the `OnRobotRG6System` `ros2_control` block — both on fake and real
hardware. The RG6 joint exists for state mirroring / robot_state_publisher
only; the real gripper is commanded outside ros2_control (via Mechanism C
URScript topic, or skipped entirely with `--no-gripper`).

This aligns with the LOCKED DECISION in `wiki/decisions.md` 2026-05-24.

---

## RTDE "Pipeline producer overflowed" spam on WSL2

**2026-05-26.** When `ur_ros2_control_node` connects to the real cabinet
from WSL2, the log floods with:
```
[UR_Client_Library:]: Pipeline producer overflowed! <RTDE Data Pipeline>
```
hundreds of times in the first seconds, sometimes ongoing.

**Why.** The driver subscribes to RTDE at 500 Hz. WSL2 is not a real-time
kernel — the FIFO scheduling warning ("Your system/user seems not to be
setup for FIFO scheduling") is the symptom. Under load (or just slow
context-switch on WSL2's vEthernet), the consumer thread can't drain
the RTDE buffer fast enough, and packets overflow.

**Impact.** Commanded motion may lag or jitter. State readback (`/joint_states`)
still works because the broadcaster publishes at controller-manager rate,
not RTDE rate — so the snapshot is fresh.

**Workarounds.**

**1. Lower controller_manager update_rate (PARTIALLY WORKS, applied).**
Edit `src/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur10e_update_rate.yaml`
from `update_rate: 500` to `update_rate: 250`. This does NOT change the
cabinet's RTDE publish rate (log still reports "Setting up RTDE communication
with frequency 500.000000") — that's hardcoded in the upstream driver.
BUT combined with `non_blocking_read=true` (set in `ur_macro.xacro`), the
driver drops rather than buffers, and the steady-state overflow rate
settles to **~8/sec measured 2026-05-26** (well below the worst-case
~250/sec). At 8/sec we're dropping ~1.6% of RTDE samples, keeping 98.4%.
Acceptable for tiny smoke tests with slow motion. `/joint_states` still
publishes cleanly.

**2. (Untried) Modify driver to plumb cabinet RTDE rate through a
parameter.** Upstream newer versions have `rtde_frequency` xacro arg /
hardware param. Our Humble version doesn't — would need a patch to
`hardware_interface.cpp` `URPositionHardwareInterface::on_configure()`
to set `driver_config.rtde_target_frequency` (need to verify the field
name in the installed `ur_client_library` header).

**3. (Untried) Pin the ROS process to dedicated CPU cores via `taskset` /
`chrt -f`. May need `cap_sys_nice` for the user.

**4. (Untried) Run the driver in a separate native Linux box (no WSL).
The "proper" answer for sustained streaming.

Decision for now: accept the ~8/sec noise for slow / small-motion smoke
tests. Revisit before sustained pickplace streaming on real hardware.

---

## `ros2 topic` CLI hangs in WSL2; rclpy direct subscriber works

**2026-05-26.** With the driver fully up and `joint_state_broadcaster`
publishing, `ros2 topic list` and `ros2 topic echo /joint_states --once`
hang indefinitely from a separate bash session, even with the same
`ROS_DOMAIN_ID`, RMW, and no `ROS_LOCALHOST_ONLY`. `ros2 daemon stop`
doesn't help.

**Workaround.** Use a direct rclpy subscriber. Saved at
`/tmp/peek_joint_states.py` for quick re-use:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# create_subscription("/joint_states", ...) then spin_once with timeout
```
This sees the topic and prints messages within ~1 s of launch.

**Suspected cause.** WSL2 mirrored-networking + FastDDS multicast
discovery races. The CLI's discovery probe uses a short timeout that
loses to the kernel re-routing latency. rclpy's `spin_once` keeps
re-running discovery so it eventually wins.

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

## WSLg RViz corrupts after many launch cycles (pink window / tiny stub window)

**2026-05-26 / 2026-05-28.** After ~8-10+ launch/relaunch cycles in a single
session, RViz stops rendering. Two observed manifestations, same root cause
(WSLg's Qt+OpenGL state corrupts):
- **Pink/purple gradient** instead of the robot model (2026-05-26).
- **Tiny non-rendering stub window** — RViz opens a small window showing only
  the app icon, no panels/3D view (2026-05-28, after many gripper-test
  relaunches). GL itself is fine (`glxinfo` still shows D3D12, GL 4.1); it's
  the Qt window/GL context that's wedged.

Bare `rviz2` (no config) also fails once wedged.

**Workaround (verified both times):** from PowerShell on Windows:
```powershell
wsl --shutdown
```
Wait ~5 s, reopen WSL terminal. WSLg state resets cleanly. RViz renders on the
next launch. **No workspace state or files lost** — only running processes die
(the ROS stack + any live hardware sessions) and the WSL kernel instance
recycles. Re-launch the stack afterward.

**Prevention / tip:** launch RViz from your OWN interactive WSL terminal, not
from a detached one-shot (`setsid nohup` / `wsl.exe -- bash -c ...`) — a
detached RViz window may not surface under WSLg. Use
[`scripts/view_rviz.sh`](../scripts/view_rviz.sh) to bring RViz up attached in
your terminal.

---

## RViz Fixed Frame "world" has no TF → robot invisible even when window renders

**2026-05-28.** Distinct from the WSLg issue above: even with a healthy RViz
window, the robot may not appear because `config/moveit.rviz` sets
**Fixed Frame = `world`**, but **nothing publishes a `world → base_link`
TF**. The SRDF defines a `world_to_base` virtual joint, but that is a MoveIt
*planning* construct — `robot_state_publisher` only knows the URDF (rooted at
`base_link`), and no `static_transform_publisher` for it exists in any launch
file. So `world` is unresolvable and the RobotModel can't be placed.

**Workarounds (either):**
- Publish the transform: `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link`
  (done automatically by [`scripts/view_rviz.sh`](../scripts/view_rviz.sh)).
- Or in RViz: Global Options → Fixed Frame → set to `base_link`.

**Proper fix (TODO):** add the static `world → base_link` publisher to
`full_stack.launch.py` so the planning frame `world` is always in TF.

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

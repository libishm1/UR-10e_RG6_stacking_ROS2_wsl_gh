# Locked decisions

## Purpose

Permanent record of design choices that shouldn't drift across sessions.
Each entry is a one-liner decision + the reasons that locked it. If a
decision is reversed, append a new entry — don't edit the old one.

Pattern borrowed from `D:\robot_ws\robots\wiki\project_management\decisions.md`.

---

## 2026-05-28 — RG6 control: digital tool-I/O PARKED, **RS485/Modbus chosen** (supersedes the 2026-05-27 binary-Tool-I/O plan)

**Decision.** Drive the real RG6 over the **UR tool-flange RS485 (Modbus
RTU)**, bridged to the host by `ur_robot_driver`'s `use_tool_communication`.
The digital tool-I/O path (`set_io` pins 16/17) is **abandoned**.

**Why — tested on hardware at the cell (2026-05-28).** Driving the tool
digital outputs tripped the cabinet's **tool-connector overcurrent
protection, twice**, and the gripper never moved:
- Sinking (NPN) → "too high **sink** current on Digital Output 0, low side"
- Sourcing (PNP) → "too high **source** current on Digital Output 1, high side"

Both polarities fault ⇒ the RG6's digital lines present a low-impedance load
because the gripper is in **RS485/URCap mode, not Teach mode** — its MCU
holds the I/O lines. The reference drivers assume a gripper *already in Teach
mode (URCap uninstalled)*, which we could not achieve from software. Full
post-mortem + the digital retry handoff:
[rg6_urcap_hardware_pitfalls.md](rg6_urcap_hardware_pitfalls.md).

RS485 is the gripper's **native** channel (the same one the URCap uses), so
it carries **no overcurrent risk** (comm bus, not a driven power line), gives
**continuous width + force + grip-detect**, and **coexists with External
Control** for arm motion. The tool-comm params match this cell's pendant
exactly (1M / Even / One / 24 V).

**Implications / what changed in code.**
- New [tests/onrobot_modbus_grip.py](../tests/onrobot_modbus_grip.py) —
  pymodbus RTU client, device_id 65, write regs 0–2 `[force×10, width×10, 1]`,
  read regs 258+ (width @9, status @10). Register map + setup:
  [rg6_rs485_modbus.md](rg6_rs485_modbus.md).
- New [scripts/launch_real_rs485.sh](../scripts/launch_real_rs485.sh) —
  full stack with `use_tool_communication:=true tool_voltage:=24` etc.;
  `full_stack.launch.py` + `ur10e_rg6_control.launch.py` thread the tool args
  through (defaults OFF, so sim/digital launches are unchanged).
- `play_pickplace.py --real-gripper` now defaults to `--gripper modbus`
  (continuous); `--gripper io` keeps the parked digital path for reference.
- [tests/onrobot_io_grip.py](../tests/onrobot_io_grip.py) kept but PARKED.

**To verify at the cell (not yet hardware-tested).** (1) the register map
holds over RTU (read actual width, confirm it tracks the fingers); (2) whether
e-Series needs the `rs485` daemon URCap or `use_tool_communication` suffices;
(3) URCap coexistence (rs485 vs OnRobot URCap) — likely the two-installation
pattern. See [rg6_rs485_modbus.md](rg6_rs485_modbus.md) "Working assumptions".

**Reference.** [tonydle/ur_onrobot](https://github.com/tonydle/ur_onrobot),
[Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot).

---

## 2026-05-26 (very late evening) — RG6 real-hardware control: REVERSED to Mechanism A/B (Tool I/O), NOT Mechanism C

**Decision.** Use **Tool digital I/O** for real-hardware gripper control —
the same approach the upstream `inria-paris-robotics-lab/onrobot_ros`
reference repo takes. Either:

- **Mechanism A** (preferred long-term): the `onrobot_interface` C++
  `ros2_control` hardware-interface plugin, BUT fix the URDF
  `<ros2_control name="OnRobotRG6System">` block to satisfy the
  plugin's `<param name="prefix">` and `<param name="model">`
  requirements that crashed it earlier
  ([known_bugs](known_bugs_and_workarounds.md#onrobot_interface-c-plugin-crashes-on-init--never-use-it)
  has the previous crash trace).
- **Mechanism B**: the `onrobot_driver` Python node — runs as a
  separate ROS node, subscribes to `/io_and_status_controller/io_states`,
  offers higher-level `grip(width, force)` services. No URDF changes.
- **Roll-our-own minimal**: a 30-line ROS helper that directly calls
  `/io_and_status_controller/set_io` to set pin 16 + reads pin 17 +
  optionally writes the analog tool voltage. Bypasses both plugin and
  separate driver.

**Why this reverses the 2026-05-24 decision (below):**

The 2026-05-24 decision picked Mechanism C (URScript topic) because A
and B were thought to be "binary open/close only". This turns out to
be **wrong** — A and B both use the OnRobot tool I/O analog voltage
channel (0-3 V for RG6 v1, 0-10 V for RG6 v2) for continuous width
control. The reference plugin source
([`onrobot_gripper.cpp`](../src/onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp))
shows this directly:

```cpp
int PIN_GRIPPER_CONTROL = 16;   // tool digital out  → command
int PIN_GRIPPER_STATE   = 17;   // tool digital in   → state
float DEFAULT_MAX_POSITION_VOLTAGE_RG6_V2 = 10.0;  // analog → width
```

It's not binary; it's continuous via the analog voltage rail.

Mechanism C (URScript topic `rg_grip(...)`) is **architecturally
unworkable from External Control's URScript-topic context.** Verified
2026-05-26 by:

1. Rebuilding `external_control.urp` on the pendant with an OnRobot RG
   node FIRST in MainProgram, External Control SECOND (the URP file is
   saved at `calibration/urp/external_control_with_onrobot_node.urp`).
2. Confirmed via SSH that the OnRobot URCap node DOES execute at URP
   start (gripper moves to its configured width during program load).
3. But `rg_grip(50, 20)` sent via `/urscript_interface/script_command`
   STILL silently no-ops — the OnRobot URCap's `rg_grip` lives in a
   Java-backed namespace tied to its own program-node execution; it's
   not reachable from URScript text arriving on the External Control
   socket.

So C requires either Path B URP-load-per-grip (slow, brittle) or
rebuilding URPs server-side (complex). Tool I/O sidesteps the entire
URCap. PolyScope just routes the tool pins to the gripper MCU — no
URCap, URScript, or URP machinery in between.

**Implications.**

- The `<plugin>mock_components/GenericSystem</plugin>` we're using for
  `OnRobotRG6System` in `ur10e_rg6.urdf.xacro` should EVENTUALLY be
  reverted to `<plugin>onrobot_interface/OnRobotHardwareInterface</plugin>`
  with the proper params, OR we go with Mechanism B (independent node)
  and keep mock_components.
- The `--real-gripper` flag in `play_pickplace.py` currently publishes
  to `/urscript_interface/script_command`. This needs to be replaced
  with a call to `/io_and_status_controller/set_io` (for Mechanism A/B/own).
- The `WAYPOINT_TOOL_CALIBRATION_M` X/Y shift we measured is
  **independent** of the gripper control mechanism — it's a TCP-frame
  calibration. Stays valid.

**Reference implementation to crib from.**
[`src/onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp`](../src/onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp).
The proven gripper-via-tool-I/O code path on this exact hardware family.

---

## 2026-05-27 — Tool I/O on our cell is BINARY only (refines 2026-05-26)

**Decision.** Scope the 2026-05-26 Tool I/O decision: on **this** cell
(UR10e e-Series + RG6 v1 + OnRobot URCap on pendant, NO Compute Box),
`set_io` to pin 16 gives **binary close/open only**. There is no
continuous-width path available via the e-Series tool flange.

**Why the 2026-05-26 entry was over-optimistic.**

The upstream `onrobot_gripper.cpp` references `analog_output_voltage`
on the tool to set width — but UR **e-Series** controllers have **no
analog output** to the tool flange (only digital out 0/1 + analog
**inputs** for sensors). The upstream code was written against either
CB-series controllers or a setup with an OnRobot Compute Box in front.
On our cell, `set_io` with `FUN_SET_ANALOG_OUT` to the tool pins is a
no-op silently — the cabinet has nothing to drive it.

**What this means in practice.**

- For the wood-block stacking demo, **binary is sufficient** — every
  pick is the same physical block, so a single close-force/close-width
  is fine. `play_pickplace.py`'s `--real-gripper` path now maps
  `width_mm < 60` → CLOSE (pin 16 HIGH), else OPEN (pin 16 LOW). The
  `--force` arg is ignored. See [`tests/onrobot_io_grip.py`](../tests/onrobot_io_grip.py).
- If a future task **needs** continuous width / variable force, the
  options are:
  1. **OnRobot Compute Box** (Modbus TCP) — the OnRobot reference
     stack expects this; gives mm-level + N-level control without
     touching the cabinet.
  2. **OnRobot RS-485 URCap** — talks Modbus over the tool RS-485
     pins directly from the ROS host. No Compute Box, but needs a
     URCap reinstall on the pendant.
  3. **Different gripper** (Robotiq 2F-85 or similar) with a
     ROS-native driver.

  Out of scope for the current demo; flag if a real task needs it.

- Pin-numbering verification needed at runtime: the third-party
  pasted analysis said `IOStates.tool_digital_input_states[0]` exists,
  but `ros2 interface show ur_msgs/msg/IOStates` shows it does NOT in
  our Humble version. Tool DI for grip-detect may appear in
  `digital_in_states` at a specific pin number — runtime-verify
  tomorrow with the driver connected:
  ```bash
  ros2 topic echo /io_and_status_controller/io_states --once | head -40
  ```
  Until verified, `OnRobotToolIOGrip.close_blocking()` uses a fixed
  1.5 s settle.

**Pendant prereq this entry locks in.** Installation → General →
Tool I/O → `Controlled by: User` — required so the OnRobot URCap
doesn't fight our `set_io` writes. This is a one-time pendant
config, separate from the URP load + Remote Control mode for
External Control.

---

## 2026-05-24 — RG6 real-hardware control: Mechanism C (URScript topic) — SUPERSEDED 2026-05-26

**Decision.** For real-hardware gripper control, use **Mechanism C**:
publish single-line `rg_grip(width_mm, force_N, ...)` URScript to
`/urscript_interface/script_command`. Do NOT use the
`onrobot_interface` C++ ros2_control plugin (Mechanism A) or the
`onrobot_driver` Python node (Mechanism B), even though both are
shipped in the reference repos.

**Why.**

1. **Continuous width and force.** A and B are effectively binary
   (open/close + low-force mode bit). The pick-and-place sequence
   needs to grip at 50 / 60 / 70 mm with controllable force; only
   C gives mm- and N-level control.
2. **No RG6 ROS boilerplate required.** C works with stock
   `ur_robot_driver` + any MoveIt-for-UR. A needs the URDF plugin
   block + `rg6_joint` + controller-manager config; B needs the
   `onrobot_driver` Python node running. C removes a whole layer of
   things that can break.
3. **URCap on pendant is the same OnRobot URCap already installed.**
   A and B require the URCap in "mounted-to-UR / pin mode" — we'd
   have to reconfigure the pendant. C uses the URCap in its default
   "URScript-callable" mode, which the cell is already set up for
   (verified 2026-05-10 on dodectest3.urp).
4. **Same code path as our verified sim runs.** `play_pickplace.py`
   already uses `/urscript_interface/script_command` for the
   gripper in `--real-gripper` mode; switching to A/B would mean a
   different code path on real hardware than the one verified in
   sim.

**Implications.**

- The `<plugin>onrobot_interface::OnRobotHardwareInterface</plugin>` block
  in [`src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro`](../src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro)
  is now DORMANT for real-hardware. It's still selected when
  `use_fake_hardware:=false`, but we'll bypass it in practice by
  using mechanism C from our scripts. Keep the URDF wiring for now
  (no need to rip it out) — leave as future cleanup if we ever
  confirm we never want A.
- The `rg6_gripper_controller` (joint_trajectory_controller) is
  still useful for SIM runs (`play_pickplace.py` default mode) so
  ghost-robot RViz shows the gripper opening/closing. Keep it.
- Width-mm ↔ angle-rad cubic in
  [`config/rg6_width_calibration.yaml`](../src/ur10e_rg6_moveit_config/config/rg6_width_calibration.yaml)
  is only used by the SIM path now — it's irrelevant to real
  hardware. Keep for sim parity.

**What changes in practice.** Nothing in the scripts —
`play_pickplace.py --real-gripper` and
`real_hw_smoke.py --yes --real-gripper` are already on path C. This
decision just locks the choice and says "don't get tempted into A or
B without a real reason".

**Reference.** [`rg6_control_mechanisms.md`](rg6_control_mechanisms.md)
for the full A/B/C comparison and the code citations.

---

## Last updated

2026-05-28 (digital tool-I/O PARKED after hardware overcurrent faults; RS485/Modbus chosen).

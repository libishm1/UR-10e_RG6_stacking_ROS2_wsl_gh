# RG6 control from ROS 2 — three mechanisms in the reference repos

> **Locked decision (2026-05-24):** this workspace uses **Mechanism C**
> (URScript topic) for real hardware. See [decisions.md](decisions.md).
> The A/B comparison below is kept for context and to explain why C won.

## Purpose

The vendored repos under `~/ur_rg6_ws/src/` ship THREE different ways to
drive the RG6 from ROS 2. They all reach the same hardware but through
different chains. This page lays them out so you can pick the right
one and understand what each requires on the pendant side.

## Confirmed facts

All three mechanisms ultimately route through the UR cabinet's tool I/O
to the RG6 mounted on the flange. The OnRobot URCap (already installed
on this pendant) is what physically actuates the gripper in every case
— the question is which interface ROS uses to talk to the URCap.

The reference code lives at:

- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_interface/` (C++ ros2_control plugin)
- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_driver/` (Python ROS 2 node)
- Any client publishing to `/urscript_interface/script_command`
  (`play_pickplace.py`, `real_hw_smoke.py`).

## Mechanism A — `onrobot_interface` (C++ ros2_control hardware plugin)

**File:** [`onrobot_interface/src/onrobot_interface.cpp`](../src/onrobot1_ros/onrobot_interface/src/onrobot_interface.cpp) (158 lines)
+ [`onrobot_gripper.cpp`](../src/onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp) (285 lines).

**How it works.** Implements `hardware_interface::SystemInterface` (the
ros2_control hardware plugin contract). On `on_activate()` it spins up
a background thread that:

1. Waits for `/io_and_status_controller/set_io` (a `ur_msgs/SetIO`
   service exposed by `ur_robot_driver`).
2. Sets tool voltage to **24 V** via `SetIO(fun=4, pin=0, state=24)`.
3. Toggles **digital out pin 16** to "wake up" the RG6 (high → low).

Then per `write()` tick of the controller manager:

| Inputs                          | Action                                            |
|---------------------------------|---------------------------------------------------|
| `position_command > position_state` | `SetIO(fun=1, pin=16, state=1)` → CLOSE       |
| `position_command < position_state` | `SetIO(fun=1, pin=16, state=0)` → OPEN        |
| `position_command == prev` (no change) | no I/O                                       |

`read()` ticks pull `analog_input2` from `/io_and_status_controller/tool_data`
and convert it to a 0–1.3 rad position via the calibrated voltage range
(0.6 V = full closed → max_voltage V = full open, scaled).

**What this means in practice.**

- ROS sees `rg6_joint` as a normal `position`-command joint — usable by
  MoveIt, joint_trajectory_controller, RViz sliders, anything.
- **But: it's effectively a binary actuator.** The write logic only
  picks open or close; there's no mm-level width control. Any
  trajectory point becomes "is this less or more than where we are?"
  → toggle the pin. The intermediate width values in a trajectory
  are ignored.
- Plus a low-force mode bit (`pin 17`) which our URDF doesn't wire up
  by default.

**Pendant prerequisites.** The OnRobot URCap must be in **"mounted to UR" /
"local tool I/O" mode** so it listens on pin 16 / pin 17 for commands
and drives the corresponding tool-I/O outputs. The URCap pendant UI
toggles this — see OnRobot RG documentation. **Does NOT use** the URCap's
URScript `rg_grip` function; only its pin-driven control surface.

**Where it shows up in our URDF.** [`ur10e_rg6.urdf.xacro`](../src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro)
selects this plugin when `use_fake_hardware:=false`:

```xml
<xacro:unless value="$(arg use_fake_hardware)">
  <plugin>onrobot_interface::OnRobotHardwareInterface</plugin>
</xacro:unless>
```

## Mechanism B — `onrobot_driver` (standalone Python node)

**File:** [`onrobot_driver/onrobot_driver/gripper_controller.py`](../src/onrobot1_ros/onrobot_driver/onrobot_driver/gripper_controller.py) (186 lines).

**How it works.** Same physical chain as Mechanism A — driver →
`SetIO` service → tool I/O pins → URCap-in-pin-mode → gripper. The
only difference is the Python class is a standalone `rclpy.Node`
(not a `hardware_interface::SystemInterface` plugin).

Exposes high-level methods (`enable()`, `disable()`, `open()`,
`close()`, `is_ready`, `opening`) that internally call the same
`SetIO(fun, pin, state)` requests.

**When to reach for it.**

- If you do NOT want a ros2_control hardware plugin (you don't want
  the gripper joint inside a controller manager), Mechanism B gives
  you the same control as a regular ROS 2 node you can spawn or kill
  independently of the arm stack.
- Same binary-open-or-close limitation as Mechanism A.

## Mechanism C — URScript topic (`/urscript_interface/script_command`)

**Files:** [`tests/play_pickplace.py`](../tests/play_pickplace.py),
[`tests/real_hw_smoke.py`](../tests/real_hw_smoke.py); the topic itself
comes from `ur_robot_driver`'s default launch.

**How it works.** Publish a `std_msgs/String` containing a single
URScript line like:

```python
rg_grip(80.0, 25.0, tool_index=0, blocking=True, depth_comp=False, popupmsg=False)
```

`ur_robot_driver` forwards it to the cabinet over port 30002. The
OnRobot URCap (in its normal "URScript-callable" mode) on the pendant
parses `rg_grip(width_mm, force_N, ...)` and actuates the gripper
through tool I/O — same physical chain as A and B, but the URCap does
the pin-toggling instead of ROS.

**What this means in practice.**

- **Real width control in mm and real force in N.** `rg_grip(50.0, 25.0, ...)`
  closes to 50 mm at 25 N. This is the only mechanism that lets you
  pick a continuous width.
- **No ros2_control gripper config required.** The URScript topic is
  auto-advertised by `ur_robot_driver`. You can launch a stock
  `ur_robot_driver` + `ur_moveit_config` (no custom URDF, no custom
  controllers) and `ros2 topic pub /urscript_interface/script_command ...`
  just works.
- **Single-line URScripts are "secondary programs"** — they do NOT
  interrupt the External Control program. If you ever wrap one in
  `def ... end`, the script runs as a "primary program" and STOPS
  External Control; you'd have to `resend_program` or Play again
  (per upstream docs).
- **Cold-boot quirk.** First `rg_grip` after a cold cabinet boot triggers
  "RG grip didn't initialize" — restart the cabinet once, then it works
  ([SESSION_CLOSE.md gotcha](../SESSION_HANDOFF.md)).

## Side-by-side comparison

| Aspect                                | A: onrobot_interface (C++)     | B: onrobot_driver (Python)     | C: URScript topic              |
|---------------------------------------|--------------------------------|--------------------------------|--------------------------------|
| ROS-side mechanism                    | ros2_control hardware plugin   | standalone rclpy.Node          | `std_msgs/String` topic        |
| Physical chain                        | UR tool I/O pins → URCap       | UR tool I/O pins → URCap       | URScript → URCap → tool I/O    |
| Width control                         | binary (open/close)            | binary (open/close)            | **continuous mm**              |
| Force control                         | bit: normal / low-force         | bit: normal / low-force        | **continuous N**               |
| URCap pendant mode required           | "mounted to UR" / pin mode     | "mounted to UR" / pin mode     | URScript-callable (default)    |
| ros2_control gripper config required  | yes (this URDF's `<ros2_control>` block) | no — it's a separate node | **no** (no RG6 config needed) |
| MoveIt-compatible gripper joint?      | yes (position command interface) | no (own API)                  | no (own API)                   |
| Position feedback                     | yes (analog_input2 → rad)      | yes (same source)              | no (URCap blocks, then returns)|
| Compatibility with this workspace's `play_pickplace.py` | partial (binary clamp) | no | **full (used by `--real-gripper`)** |
| Used in our `ur10e_rg6_moveit_config` | yes (via URDF `<plugin>`) when `use_fake_hardware:=false` | no | yes (in scripts) |

## When to use which

| Goal                                                                 | Pick |
|----------------------------------------------------------------------|------|
| Replay the URScript program literally (widths in mm matter)          | **C** |
| Plan a MoveIt trajectory that includes the gripper as a planning group | **A** (binary actuator inside MoveIt) |
| Quick ROS 2 node to open/close from elsewhere with no boilerplate    | **B** or **C** |
| Avoid configuring the URCap "pin mode" (just use it as installed)    | **C** |
| Avoid touching ros2_control entirely                                 | **B** or **C** |
| Need position feedback in `/joint_states`                             | **A** (B has its own callback, C has none) |

For this workspace's pick-and-place use case (varying widths between
1 mm and 153 mm with controllable force), **C is the right answer** —
and is what `play_pickplace.py --real-gripper` and
`real_hw_smoke.py --real-gripper` already do.

A is wired into our URDF as the real-hardware ros2_control fallback,
but **with the binary-actuator caveat** — its width control is on/off
relative to the previous command. If you switch to A as primary, the
URScript widths in `play_pickplace.py` lose their granularity.

## Why this matters for the "do I need the RG6 boilerplate?" question

The user asked whether a vanilla ROS 2 + UR setup can control the
gripper without the custom RG6 config. The answer depends on
mechanism:

| Mechanism | Needs custom RG6 ROS config? |
|-----------|------------------------------|
| A         | YES — the `onrobot_interface` plugin must be selected in URDF; the rg6_joint must be defined; the controller manager must spawn a gripper controller |
| B         | YES — must run the `onrobot_driver` node; no rg6_joint needed |
| **C**     | **NO** — stock `ur_robot_driver` is sufficient; no rg6_joint, no controller, no custom URDF |

So if you want **arm via vanilla ROS 2 + gripper via real hardware**,
use Mechanism C. That's the path `real_hw_smoke.py --yes --real-gripper`
takes — and it's what we documented in
[`real_hw_connection.md`](real_hw_connection.md).

## Sources

- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_interface/src/onrobot_interface.cpp`
- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp`
- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_driver/onrobot_driver/gripper_controller.py`
- `~/ur_rg6_ws/src/onrobot1_ros/onrobot_description/urdf/onrobot_ros2_control.urdf.xacro`
- `~/ur_rg6_ws/src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro`
  (where mechanism A is wired)
- [Upstream Inria fork](https://github.com/inria-paris-robotics-lab/onrobot_ros) (`ros2` branch — what `onrobot1_ros` was forked from)
- [UR ROS 2 driver script_code docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/script_code.html) — the URScript topic semantics
- [Universal_Robots_ROS_Driver issue #77](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/77) — `rg_grip` via URScript topic discussion

## Last updated

2026-05-24.

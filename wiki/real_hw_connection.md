# Real-hardware connection — ROS 2 driver path

## Purpose

Describe how this workspace talks to the physical UR10e + RG6 cell: which
TCP ports, which topics, which URCaps, which side initiates each
connection. For the SFTP+Dashboard alternative used by `D:\robot_ws`, see
[Path B vs ROS 2 driver](path_b_vs_ros_driver.md).

## Confirmed facts

- Cell address: laptop `192.168.1.35/24`, cabinet `192.168.1.100/24`,
  no gateway. Source: `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md`.
- Cabinet PolyScope `5.24.0.1219432`, robot s/n `20255201551`.
- Cabinet ships with **all 5 TCP services blocked** by default; must be
  enabled at pendant Settings → Security. Discovered 2026-05-09, before
  the laptop could reach anything.
- The OnRobot URCap (RG6 driver) is already installed on this pendant
  and verified working on 2026-05-10 (`dodectest3.urp` run, 4×
  `rg_grip` cycles).
- The **External Control URCap** is the URCap that
  `ur_robot_driver` requires for live trajectory streaming. It is a
  DIFFERENT URCap from the OnRobot one — both must be installed for the
  ROS 2 driver path + gripper to work in the same session.
- `/urscript_interface/script_command` (a `std_msgs/msg/String` topic)
  is **auto-advertised by `ur_robot_driver`'s default launch files** —
  no custom config needed. Anything published is forwarded to the
  cabinet's secondary client interface (port 30002) and runs there.

## TCP ports and direction

| Direction | Port  | Protocol | Initiator                      | Purpose                                                   |
|-----------|-------|----------|--------------------------------|-----------------------------------------------------------|
| WSL→cab   | 29999 | TCP      | dashboard client               | `load`, `play`, `stop`, `programState`, `robotmode`       |
| WSL→cab   | 30001 | TCP      | (rare)                         | primary client interface                                  |
| WSL→cab   | 30002 | TCP      | URScript topic / direct script | secondary client (gripper URScript via OnRobot URCap)     |
| WSL→cab   | 30004 | TCP      | RTDE client                    | state out @ ≤500 Hz, command in                           |
| **cab→WSL** | **50001** | **TCP** | **External Control URCap** | **reverse channel: URScript stream (the bring-up gate)**  |
| **cab→WSL** | **50002** | **TCP** | **External Control URCap** | **reverse channel: trajectory points**                    |
| WSL→cab   | 22    | TCP      | SSH/SFTP (only Path B uses)    | upload .urp/.script to `/programs/`                       |

The **reverse channel** is the only direction the cabinet initiates. It
opens after the operator presses Play on the External Control program.
Every WSL2 networking gotcha (mirrored vs bridged vs NAT) reduces to:
"can the cabinet dial back into WSL on 50001/50002?".

## Topics and actions used by this workspace

| Resource                                       | Provided by                              | Used by                                       |
|------------------------------------------------|------------------------------------------|-----------------------------------------------|
| `/move_action` (MoveGroup action)              | move_group (this workspace)              | `play_pickplace.py`, `real_hw_smoke.py`, RViz |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | ur_controllers (driver)   | move_group (arm execution)                    |
| `/rg6_gripper_controller/joint_trajectory`     | joint_trajectory_controller (our config) | sim gripper path (mock or driver-mock)        |
| `/urscript_interface/script_command`           | ur_robot_driver (always)                 | real gripper path (rg_grip via URCap)         |
| `/joint_states`                                | joint_state_broadcaster (driver)         | TF, MoveIt, scripts                           |
| `/io_and_status_controller/dashboard/*`        | ur_controllers (driver)                  | dashboard ops from ROS                        |

## What is mandatory vs what is convenience

| Layer                                         | Arm-only test | Arm + gripper test (real HW) | Why                                                                                            |
|-----------------------------------------------|---------------|------------------------------|------------------------------------------------------------------------------------------------|
| `ur_robot_driver`                             | **required**  | **required**                 | streams trajectories via External Control URCap, advertises URScript topic                     |
| Some MoveIt for UR (e.g. `ur_moveit_config`)  | **required**  | **required**                 | needs `/move_action` to plan                                                                   |
| `ur10e_rg6_moveit_config` (this repo)         | optional      | optional                     | a convenience overlay — combines UR + RG6 + floor; can swap in vanilla `ur_moveit_config`      |
| Custom RG6 URDF + `rg6_gripper_controller`    | not needed    | NOT needed for real path     | The URScript topic invokes the OnRobot URCap on the pendant — no ROS-side gripper config needed |
| RG6 calibration yaml (`rg6_width_calibration.yaml`) | not needed | not needed for real path     | only the sim path uses it (width-mm → rad cubic). The real `rg_grip()` takes width in mm directly. |
| External Control URCap (pendant)              | **required**  | **required**                 | accepts driver's reverse connection on 50001/50002                                              |
| OnRobot URCap (pendant)                       | not needed    | **required**                 | interprets `rg_grip(...)` URScript calls and drives the gripper via tool I/O                    |
| SSH key on pendant                            | not needed    | not needed                   | only Path B uses SFTP. The ROS 2 path uses no SSH at all.                                       |

**Practical reading:** if you take the workspace to another cell with
the same UR model, you only need the OnRobot URCap + External Control
URCap on the pendant and the cabinet IP. The custom RG6 ROS config is
nice-to-have for sim testing but irrelevant to the real-hardware path.

## Standard launch (real hardware)

```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
    use_fake_hardware:=false \
    robot_ip:=192.168.1.100
```

After ~20 s the driver advertises `/move_action` and waits for the
reverse connection. **Press Play on the pendant** External Control
program. Driver logs:

```
[ur_robot_driver]: Robot connected to reverse interface
[ur_robot_driver]: Ready to receive control commands
```

Then run motion clients (`real_hw_smoke.py`, `play_pickplace.py`).

## Vanilla equivalent (no RG6 boilerplate)

If you don't have / don't want this workspace's overlay:

```bash
# Arm only (mirrors our full_stack.launch.py minus RG6 bits)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e robot_ip:=192.168.1.100 \
    kinematics_params_file:=$HOME/ur10e_calibration.yaml \
    launch_rviz:=false

# Plus MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e

# Drive gripper from any shell (no RG6 config required)
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String \
  "{data: 'rg_grip(80.0, 25.0, tool_index=0, blocking=True, depth_comp=False, popupmsg=False)\\n'}"
```

This proves the gripper path is RG6-ROS-config-free. The URScript topic
plus the URCap-on-pendant is the only chain that has to exist.

## Pre-flight + smoke test sequence

1. `tests/check_real_hw_network.sh 192.168.1.100` — ICMP, TCP probes
   on 29999/30001/30002/30003/30004, Dashboard handshake, reverse-channel
   listener readiness.
2. `python3 tests/real_hw_smoke.py` — dry-run (prints intent only).
3. `python3 tests/real_hw_smoke.py --yes --no-gripper` — arm only,
   real hardware, no gripper boilerplate touched at all.
4. `python3 tests/real_hw_smoke.py --yes --real-gripper --force 25` —
   add the URScript gripper path.
5. `python3 tests/play_pickplace.py --real-gripper --force 25 --max 1`
   — one full pick-place cycle at low force.
6. `python3 tests/play_pickplace.py --real-gripper --force 40` — full
   10-cycle program at normal force.

## Known issues

### Bare URScript on port 30002 + URCap functions: CABINET CRASH

**Verified 2026-05-26:** sending a single-line `rg_grip(width, force)`
via a raw TCP socket to `192.168.1.100:30002` (no URCap preamble loaded)
**caused a URCap error and forced a cabinet restart.** The cabinet had
just booted and the OnRobot URCap had enumerated the gripper cleanly
(pendant Date Log: `OnRobot Devices: Quick Changer + RG6 + [0.0, ...]`),
but the first ad-hoc `rg_grip` send corrupted URCap state.

Previously (per D:\robot_ws notes) this was described as "silently
ignored." The 2026-05-26 observation is stronger: it can actively
break the URCap session.

**Rule:** never send `rg_grip()` (or any other URCap-defined function)
through a path that doesn't have the URCap preamble loaded. The URCap
preamble is injected by PolyScope only when a `.urp` is loaded. The
safe paths are:

1. **Path B** (recommended for ROS-free testing): SFTP+Dashboard deploy
   a `.urp`+`.script` pair. See [`path_b_vs_ros_driver.md`](path_b_vs_ros_driver.md)
   and `D:\robot_ws\robots\outputs\2026-05-10\path_b\urp_deploy.py`.
2. **ROS driver path**: load `external_control.urp` on the pendant,
   press **Play**, then use `/urscript_interface/script_command`. The
   driver forwards via port 30002 but PolyScope is now in a program
   context that has the URCap preamble loaded, so `rg_grip()` resolves.

**Never** open a raw socket to 30002 and send `rg_grip` lines without
one of the above contexts loaded. The cabinet will likely error and
need a restart.



- **URScript topic stops the External Control program when the message
  is a wrapped `def...end` program.** Single-line statements (like our
  `rg_grip(...)`) are sent as "secondary programs" and do NOT
  interrupt. If you ever wrap a multi-line script and the arm stops,
  you must call the `resend_program` service or press Play again.
  Source: [upstream docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/script_code.html).
- **OnRobot URCap cold-boot quirk.** First Play after a cold cabinet
  boot triggers `RG grip didn't initialize` and the cabinet shuts down.
  Workaround: cold-boot → immediately restart → then Play. Source:
  `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md`.
- **WSL2 mirrored mode loses UDP multicast receives.** Doesn't affect
  the UR driver (unicast TCP only), does affect multi-machine DDS
  discovery. See [WSL2 networking deep-dive](../docs/WSL2_UR10e_NETWORKING.md).
- **Two URCaps must coexist on the pendant.** External Control (from UR)
  AND OnRobot (RG6 driver). Don't uninstall either.

## Last updated

2026-05-24.

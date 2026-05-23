# UR10e + RG6 — Session Handoff

Last updated: 2026-05-23. Read this first; it covers the current state and how
to pick up where we left off.

## CHECKPOINT — 2026-05-23 (mid-session)

Most-recent verified milestone: **`play_pickplace.py` ran 10/10 boxes** in sim
with pre-spawn, attach-on-pick, detach-on-place, gripper-orientation-preserved
stacking. Boxes were placed with the gripper's release orientation (not
identity), stacked at z=0.021/0.053/0.085/0.117 m, on a small pedestal at
z=0.005 sized to cover the place area only (not the pick row).

In-progress changes triggered by user request right after this milestone:

1. **Box centroid convention** — switch from "centroid at box_h/2 above pose"
   to "centroid AT pose". Pre-spawn z values are now lifted from the URScript
   pick TCP z (WP_2.z=0.029 for top stack, WP_42.z=-0.001 for bottom stack).
   Detach z is now `wp_z` directly (no `- BOX_H/2` offset).
2. **Pedestal** — raised so its top sits at z=0.020 (just below the lowest
   place-box bottom at z=0.021 with the new convention).
3. **RViz Scene Alpha** — 0.9 → 1.0 so place stacks render opaque.
4. **HOME joint angles updated** — set to
   `[1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]`
   (90°, -90°, -90°, -90°, 90°, 90°) across:
   - `play_pickplace.py` HOME_Q (was already this)
   - `ur10e_rg6.srdf` `home` group_state (was 0, -90°, 0, -90°, 0, 0)
   - `initial_positions.yaml` (was 0, -90°, 0, -90°, 0, 0)

   This puts the arm on the other side of the cell using the existing
   URDF orientation. An earlier attempt rotated the URDF base 180° about
   Z; that was reverted — the URDF stays at `rpy="0 0 0"` and rotation is
   expressed purely through home joint angles.

Companion file: `tests/gripper_test.py` is a minimal gripper-only test with
optional ±6 cm joint swing near HOME (uses OMPL RRTConnectkConfigDefault to
avoid Pilz PTP edge-cases on tiny moves). Use `--no-arm` to skip the arm.


## TL;DR — start everything from a cold WSL boot

```bash
wsl -d Ubuntu-22.04
source /opt/ros/humble/setup.bash
source ~/ur_rg6_ws/install/setup.bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
```

That brings up: ur_robot_driver (fake hardware) → robot_state_publisher → JTC
gripper controller → move_group (OMPL + Pilz) → RViz with MoveIt panel. Give
it ~20 s.

If the gripper controller didn't auto-spawn (occasional timing race), manually:
```bash
ros2 param set /controller_manager rg6_gripper_controller.type joint_trajectory_controller/JointTrajectoryController
ros2 run controller_manager spawner rg6_gripper_controller \
  --param-file ~/ur_rg6_ws/install/ur10e_rg6_moveit_config/share/ur10e_rg6_moveit_config/config/rg6_jtc.yaml \
  --controller-manager-timeout 10
```

---

## What's been built

### Directory layout

```
~/ur_rg6_ws/
├── src/
│   ├── Universal_Robots_ROS2_Description/     UR meshes + our combined xacro
│   │   └── urdf/ur10e_rg6.urdf.xacro          ★ combined model (UR + RG6 + floor)
│   ├── Universal_Robots_ROS2_Driver/          UR driver (ur_robot_driver, ur_bringup…)
│   ├── onrobot1_ros/                          OnRobot RG meshes + reference xacro
│   ├── ur10e_rg6_moveit_config/               ★ our MoveIt 2 config package
│   │   ├── config/
│   │   │   ├── ur10e_rg6.srdf                 planning groups + collision pairs
│   │   │   ├── joint_limits.yaml              ACTIVE joints only (fixed/mimic ignored)
│   │   │   ├── pilz_cartesian_limits.yaml     LIN/CIRC Cartesian limits
│   │   │   ├── ompl_planning.yaml             OMPL pipeline + per-group planner configs
│   │   │   ├── pilz_planning.yaml             Pilz + AddTimeOptimalParameterization adapter
│   │   │   ├── kinematics.yaml                LMA IK on ur_manipulator only
│   │   │   ├── moveit_controllers.yaml        FollowJointTrajectory → both controllers
│   │   │   ├── rg6_jtc.yaml                   gripper JTC params
│   │   │   ├── initial_positions.yaml         home + safe gripper boot
│   │   │   └── moveit.rviz                    RViz config with MotionPlanning panel
│   │   └── launch/
│   │       ├── full_stack.launch.py           ★ one-shot bring-up
│   │       ├── move_group.launch.py           move_group only
│   │       └── moveit_rviz.launch.py          RViz only
│   ├── moveit2/                               MoveIt 2 source (mostly unused — apt has these)
│   ├── ur_msgs/  ur_client_library/
│   └── …
├── docker/                                    Dockerfile + compose for Windows pkg
├── grasshopper/                               GH Python script + .ghx for Rhino
├── tests/                                     ★ verification + demo scripts (below)
└── SESSION_HANDOFF.md                         this file
```

### Planning groups (in SRDF)

| Group              | Joints                      | Use for                                       |
|--------------------|-----------------------------|-----------------------------------------------|
| `ur_manipulator`   | 6 UR (chain to tool0)       | Arm-only moves, **LIN/CIRC**, gumball drag    |
| `rg6_gripper`      | rg6_joint                   | Gripper-only, **no IK / no gumball**          |
| `arm_with_gripper` | chain + rg6_joint + mimics  | Combined moves, PTP only (LIN can't — no IK) |

### Planning pipelines available

- **OMPL** (default, `RRTConnectkConfigDefault`, `RRTstarkConfigDefault`, etc.)
- **Pilz Industrial Motion Planner** with PTP, LIN, CIRC

### Controllers

| Controller                            | Type                  | Active joints     |
|---------------------------------------|-----------------------|-------------------|
| `scaled_joint_trajectory_controller`  | ur_controllers/SJTC   | 6 UR              |
| `rg6_gripper_controller`              | JointTrajectoryCtrl   | rg6_joint         |

The gripper accepts `JointTrajectory` on `/rg6_gripper_controller/joint_trajectory`
**and** `FollowJointTrajectory` action — MoveIt uses the latter, scripts the former.

---

## What works — verified headlessly

| Test script                  | Result |
|------------------------------|--------|
| `tests/test_groups.py`       | OMPL  3/3 PASS across all groups |
| `tests/test_pilz_groups.py`  | Pilz PTP 3/3 PASS across all groups |
| `tests/test_pilz_repeat.py`  | Pilz PTP `arm_with_gripper` 5/5 |
| `tests/test_pilz_hammer.py`  | Pilz PTP `arm_with_gripper` 20/20, varied gripper widths |
| `tests/test_pilz_rviz_style.py` | PTP 5/5; LIN 1/2 (expected — see below) |
| `tests/test_bridge_endpoints.py` | All bridge endpoints healthy |
| `tests/test_floor.py`        | Floor blocks below-z=0 plans ✅ |
| `tests/test_moveit.py`       | IK PASS, plan+execute PASS |
| `tests/test_collision_v2.py` | Collision detection PASS |
| `tests/bench_ik.py`          | LMA IK median 1.6 ms |

### Demo scripts (drive the arm visibly in RViz)

| Script                          | What it does |
|---------------------------------|--------------|
| `tests/demo_full_safe.py`       | 4-waypoint slow arm wave + 4-stage gripper cycle |
| `tests/gripper_demo.py`         | Gripper-only open/half/closed/safe cycle |
| `tests/send_trajectory.py`      | Single 4-waypoint UR trajectory |
| `tests/demo_arm_and_gripper.py` | Combined arm+gripper |
| `tests/play_pickplace.py`       | Replays an 80-waypoint URScript pick-and-place (HAS TCP_TODO) |

---

## Known issues / non-trivial gotchas

### 🟡 `play_pickplace.py` TCP offset (NEW)
URScript pose vectors are at the **gripper tip** (TCP at `rg6_tcp`, 241 mm beyond
`tool0` along local Z). The script currently sends them as `tool0` goals, so
every waypoint is 241 mm too close to the table → `NO_IK_SOLUTION` on first
pick. **Fix:** transform each `(x,y,z)` by adding `(0,0,0.241)` rotated by the
goal quaternion's tool-Z axis. About 15 lines of math in `_send`. Listed as
TCP_TODO in the script's header.

### 🟡 Pilz LIN/CIRC do NOT work on `arm_with_gripper`
That group is a chain + extra joints + link decls — **not a pure chain** → no
IK solver loads → Pilz LIN refuses with `No solver for group arm_with_gripper`.
**Workaround:** for Cartesian moves, switch the Planning Group dropdown to
`ur_manipulator` — LIN/CIRC work there. For combined arm+gripper, use **PTP**
or OMPL.

### 🟡 RViz Pilz dropdown UX
After switching Planning Pipeline to Pilz in the **Context** tab, you MUST
also pick **PTP/LIN/CIRC** in the algorithm dropdown right below it. Empty
planner_id → `No ContextLoader for planner_id '' found`. RViz doesn't honour
`default_planner_config` from YAML. This is a long-known MoveIt UX gap, not
a regression. Always set both dropdowns when using Pilz.

### 🟡 The orange ghost on `arm_with_gripper` doesn't drag with a gumball
Because the union group has no IK solver (above), there's no Cartesian gumball
for it. Use **Joints** tab sliders OR named states (`home`, `up`, `open`,
`closed`, `safe`) OR switch to `ur_manipulator`.

### 🟢 Fixed: `pilz_cartesian_limits.yaml` overwrote `joint_limits.yaml`
This was the **real cause** of the long-running `map::at` mystery. Both files
live under the `robot_description_planning` parameter namespace; loading them
as separate dicts in the launch file caused the second one to wipe the first.
move_group.launch.py now merges them in Python before passing. moveit2 issue
#1691; not fixed upstream in Humble.

### 🟢 Fixed: Gripper finger geometry
The original xacro had the RG6 as two prismatic joints with only the rubber
pad rendering. Now uses the upstream `onrobot_description` parallelogram
linkage: bracket → body → moment_arm + truss_arm + finger_tip + flex_finger
per side, mirrored via 180° Z rotation on `finger_2_origin`, all driven by
the single revolute `rg6_joint` (0=closed, 1.3 rad=full open).

### 🟢 Fixed: Floor collision
A 4×4×0.01 m floor link is now part of the URDF, fixed-jointed to `world` at
z=-0.005 m. MoveIt collision detection blocks below-ground plans.

### 🔴 Out of scope (would need bigger work)
- **CHOMP planner** segfaults on the gripper mimic linkage. Removed from
  pipelines. Patch needed in `chomp_optimizer.cpp` to skip mimic joints.
- **STOMP planner** not packaged for ROS 2 Humble at all.
- **Pilz PR #2943** (proper fix for the duplicate-timestamp issue on Pilz
  sequences with blend) is in `moveit2` `main` but not back-ported to Humble.
  Current workaround: `AddTimeOptimalParameterization` request adapter on
  Pilz config. Works for our use.
- **IKFast for UR**: `pick-ik` would give 5-10× IK speedup over LMA.
  Requires `sudo apt install ros-humble-pick-ik` (sudo not available in
  current session). Already documented in `kinematics.yaml`.

---

## Common operations

### Drive the arm in RViz (the visual way)
1. Context tab → Planning Pipeline = `ompl` (default) or `pilz_industrial_motion_planner`
2. If Pilz: Algorithm dropdown right below → `PTP` (or `LIN` for ur_manipulator group)
3. Planning Group = `ur_manipulator` for Cartesian gumball; `arm_with_gripper` for joint sliders or named states
4. Displays → MotionPlanning → Planning Request → Query Goal State (ON for ghost)
5. Drag gumball / joint slider / pick stored state
6. Planning tab → Plan & Execute (velocity scaling ≤ 0.15)

### Drive the gripper from CLI
```bash
ros2 topic pub --once /rg6_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [rg6_joint], points: [{positions: [0.5], time_from_start: {sec: 2}}]}"
```
Joint position range: 0 (closed) → 1.3 (full open). Linear approximation:
`width_mm × 0.008125 = radians`.

### Diagnostic commands
```bash
ros2 control list_controllers          # both should be 'active'
ros2 node list                          # /move_group, /controller_manager, /rviz2
ros2 topic echo --once /joint_states    # current arm + gripper positions
ros2 action list                        # /move_action, /follow_joint_trajectory
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK ...
```

### Restart only one piece
```bash
pkill -9 -f move_group           # restart move_group only
pkill -9 -f rviz2                # restart RViz only
pkill -9 -f ros2_control_node    # restart controllers (kills ur_control)
```
Then relaunch the relevant `ros2 launch` line.

### Full reset
```bash
# From PowerShell on Windows
wsl --shutdown
# Wait 5s, reopen WSL terminal
wsl -d Ubuntu-22.04
# Re-run the launch
```

---

## Networking (real hardware later)

`~/.wslconfig` on Windows host has `networkingMode=mirrored` enabled. This is
the recommended way to reach a physical UR10e on the LAN from WSL2.
After any edit to `.wslconfig`, run `wsl --shutdown` and reopen — see
`docker/NETWORKING.md` for the full guide.

When ready for real hardware:
```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
    use_fake_hardware:=false  # plus robot_ip arg in onrobot launch
```
Start with velocity scaling 0.10, hand on E-stop, no people in the work
envelope.

---

## Outside the WSL workspace

| Location | Contents |
|---|---|
| `\\wsl$\Ubuntu-22.04\home\libi\ur_rg6_ws\` | The workspace (same as `~/ur_rg6_ws/` in WSL) |
| `C:\Users\libish m\.wslconfig` | WSL config (mirrored networking) |
| `C:\Users\libish m\.claude\projects\…\memory\` | Persistent memory for next Claude session |

## For next session

If picking this up in a new conversation, start by:
1. Reading this file
2. `ros2 launch ur10e_rg6_moveit_config full_stack.launch.py`
3. Try the verification scripts in `tests/` to confirm everything still works

### RG6 gripper — calibrated and aligned with upstream Inria driver

The `onrobot1_ros` package in `src/` is a fork of
`inria-paris-robotics-lab/onrobot_ros` (`ros2` branch). The convention is:

- **Master joint**: `rg6_joint`, revolute, axis 0/1/0, range **0–1.3 rad**.
- **Sign**: 0 = full open, 1.25 = closed.
- **Real-hardware HW interface**: `onrobot_interface::OnRobotHardwareInterface`,
  reads UR tool I/O voltage, scales `pourcent_pos * 1.3` rad. Already wired
  into our xacro via `<xacro:unless value="$(arg use_fake_hardware)">`.
- **Mimic chain**: 5 mimic joints (truss_arm + finger_tip × 2 sides + mirror)
  all with multiplier 1.

### Width-mm → angle-rad mapping (calibrated, cubic)

Run `tests/calibrate_rg6_width.py` once after any URDF change. It sweeps
rg6_joint 0..1.3 rad, queries TF between the two flex_finger pads, fits a
cubic, and writes the lookup to `config/rg6_width_calibration.yaml`. Our
scripts (`play_pickplace.py`, GH `ur10e_rg6_gh.py`) import the cubic.

Approximate values for quick reference:

| Width (mm) | rg6_joint (rad) |
|------------|-----------------|
| 153 (open) | 0.00            |
| 130        | 0.30            |
| 110        | 0.50            |
| 90         | 0.65            |
| 70         | 0.81            |
| 60         | 0.87            |
| 50         | 0.94            |
| 30         | 1.06            |
|  1 (closed)| 1.25            |

URDF `initial_value` = **0.77 rad ≈ 70 mm safe open** (was 0.08 = 150 mm).

### Real-hardware path (when ready)

You don't have an OnRobot Compute Box. There are still TWO ways to drive
the gripper from ROS 2; we recommend the URScript path because it works
immediately with the OnRobot URCap that's already installed on your UR
pendant.

- **A. URScript topic (RECOMMENDED — zero extra setup)** — publish
  `rg_grip(width_mm, force_N)` strings to
  `/urscript_interface/script_command` (already advertised by
  ur_robot_driver). The OnRobot URCap on the pendant interprets the call
  and drives the gripper via UR tool I/O. No Compute Box, no extra URCap
  configuration. **`play_pickplace.py --real-gripper`** uses this path
  (default is sim).
- **B. Inria analog-pin scheme** — what the `OnRobotHardwareInterface`
  plugin in our URDF does. Requires the URCap to be configured to put
  current width on an analog output pin and watch a digital input pin
  for open/close. Works but needs pendant-side URCap config. We have
  this wired (xacro switches when `use_fake_hardware:=false`).
- **C. Modbus TCP via OnRobot Compute Box** — clean but you don't have
  the Compute Box, so skip.

### `play_pickplace.py` — what it does now

Plays the user's URScript pick-and-place program with:
- Arm via Pilz LIN (auto-fallback to PTP on tight Cartesian goals)
- Gripper via JointTrajectory (sim) or URScript `rg_grip()` (real)
- **10 boxes pre-spawned** at pick positions (5 top + 5 bottom — pass-1 picks the top stack, pass-2 picks what's left)
- **Pedestal** at z=0.005 under the place area so the bottom box doesn't fall through
- Boxes get **attached to `rg6_tcp`** during transit (gripper carries them visually)
- Placed boxes use the **gripper's orientation at release** (so they keep the tilt the gripper had at the place pose)
- Successive places at the same XY (URScript design) **stack** correctly at z=0.021, 0.053, 0.085, 0.117 m

Common gotchas / what NOT to do:
- DON'T publish a custom `AllowedCollisionMatrix` as a planning-scene diff — it REPLACES the SRDF-defined ACM, wiping all adjacent-link disables (you'll get "shoulder_link colliding with base_link_inertia" errors and IK failures everywhere). The current script avoids this entirely.
- DON'T reset the planning scene with `is_diff=False` — same problem. Use a targeted CollisionObject REMOVE list (see `clear_my_boxes`).
- If a movel fails with `NO_IK_SOLUTION` on the first move, restart `move_group` — likely a corrupted ACM from a previous run.

### `play_pickplace.py --real-gripper`

When you're physically connected to the UR10e and the URCap is loaded:

```bash
python3 ~/ur_rg6_ws/tests/play_pickplace.py --real-gripper --force 40 --max 1
```

This:
- Plans all the arm moves through MoveIt exactly the same way as sim
- Publishes `rg_grip(width_mm, force_N)` URScript strings to
  `/urscript_interface/script_command` for every gripper step
- The URCap on the pendant executes the rg_grip call, blocking until done

The URScript that gets published is the *exact* same string the original
.urp program runs — verified by `ros2 topic echo
/urscript_interface/script_command`. Example:

```
rg_grip(70.0, 40.0, tool_index=0, blocking=True, depth_comp=False, popupmsg=False)
```

Modbus register map (either path can MONITOR width via this):

| Addr | Name | R/W | Unit |
|------|------|-----|------|
| 0 | TARGET_FORCE | W | 0.1 N (RG6: 0–1200) |
| 1 | TARGET_WIDTH | W | 0.1 mm (RG6: 0–1600) |
| 2 | CONTROL | W | 1=grip, 8=stop, 16=grip_with_offset |
| 267 | ACTUAL_WIDTH | R | 0.1 mm |
| 268 | STATUS | R | bits: 0=busy, 1=grip-detected, 6=safety |

### Open follow-ups

- **`play_pickplace.py` TCP offset** is FIXED — uses `TCP_OFFSET_M = 0.241`
  with a quaternion-rotated offset. Verified 20/20 cycles complete in sim.
- **Width calibration** is FIXED — cubic fit from
  `rg6_width_calibration.yaml`; scripts use it.
- **`pick-ik`** for faster IK still pending: `sudo apt install ros-humble-pick-ik`.
- **Force-command interface**: none of the actively maintained Humble RG6
  drivers expose `<command_interface name="effort">`. Force is a ROS
  parameter / service. If you want MoveIt-native force, switch the gripper
  controller from `JointTrajectoryController` to
  `gripper_controllers/GripperActionController` — that exposes
  `GripperCommand` action whose `max_effort` field is honoured.

## Memory saved for the next Claude

- `feedback_motion_speeds.md` — safe-speed defaults policy
- (this session) Pilz config gotcha (joint_limits + cartesian_limits merge)
  worth saving as a project memory too.

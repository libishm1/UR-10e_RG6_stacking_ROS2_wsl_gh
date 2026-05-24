# Launch files

## Purpose

Inventory + usage reference for every `.launch.py` in this workspace.
The naming convention is consistent across packages: each launch file
maps to one bring-up scope, and `full_stack.launch.py` is the
one-shot for "everything I need to start working in RViz".

## Quick-reference table

| Launch file | Package | Brings up | When to use |
|---|---|---|---|
| [`full_stack.launch.py`](../src/ur10e_rg6_moveit_config/launch/full_stack.launch.py) | `ur10e_rg6_moveit_config` | UR control + gripper controller + move_group + RViz | **First choice for normal use.** Single command, full sim or real-hw. |
| [`move_group.launch.py`](../src/ur10e_rg6_moveit_config/launch/move_group.launch.py) | `ur10e_rg6_moveit_config` | move_group only | When you already have `ur_control.launch.py` running and just need MoveIt on top. |
| [`moveit_rviz.launch.py`](../src/ur10e_rg6_moveit_config/launch/moveit_rviz.launch.py) | `ur10e_rg6_moveit_config` | RViz only with the MotionPlanning panel | Restart RViz after a config edit without disturbing controllers / move_group. |
| [`demo.launch.py`](../src/ur10e_rg6_moveit_config/launch/demo.launch.py) | `ur10e_rg6_moveit_config` | Stock MoveIt demo (mock controllers, no UR driver) | Pure planning experiments, no `ur_robot_driver` overhead. |
| `ur10e_rg6_control.launch.py` | `onrobot_description` (vendor) | `ur_robot_driver` + ros2_control + URDF + RSP | Included by `full_stack.launch.py` — rarely called directly. |
| `view_ur10e_rg6.launch.py` | `onrobot_description` (vendor) | URDF + joint_state_publisher_gui + RViz `view_robot.rviz` | Sanity-check the URDF/SRDF visually with no controllers. |
| `test.launch.py` / `test_gazebo.launch.py` | `onrobot_description` (vendor) | Test rigs (Gazebo not configured) | Ignore — vendor scratch. |

## Detailed: `full_stack.launch.py`

The canonical entry point. One launch handles four sub-systems via a
timed staircase:

```
t=0    ur control (UR driver + ros2_control + RSP) — includes onrobot wrapper
t=10   rg6_gripper_controller spawner (joint_trajectory_controller)
t=12   move_group (OMPL + Pilz, with merged joint+cartesian limits)
t=15   RViz with the MotionPlanning panel
```

### Args

| Arg | Default | Notes |
|---|---|---|
| `use_fake_hardware` | `true` | `true` = mock_components (no robot). `false` = `ur_robot_driver` against the real cabinet. |
| `robot_ip` | `127.0.0.1` | Ignored when `use_fake_hardware:=true`. Set to your UR's LAN IP for real hw (e.g. `192.168.1.100`). |

### Examples

```bash
# Sim (default): mock controllers, no robot needed
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py

# Real UR10e cell — must press Play on the External Control program after launch
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
    use_fake_hardware:=false \
    robot_ip:=192.168.1.100
```

### What gets advertised

After `ALL_UP` (~15-20 s), these are live:

- `/move_action` (MoveGroup action) — used by `play_pickplace.py`, `real_hw_smoke.py`, RViz
- `/scaled_joint_trajectory_controller/follow_joint_trajectory` — UR arm trajectory
- `/rg6_gripper_controller/joint_trajectory` — gripper (sim path)
- `/urscript_interface/script_command` — gripper via URCap on real hw (auto-advertised by `ur_robot_driver`)
- `/joint_states` @ 500 Hz — all 7 joints (6 UR + rg6_joint)
- `/monitored_planning_scene`, `/io_and_status_controller/*`, etc.

### Dual-RViz fix

Without intervention, `full_stack.launch.py` would spawn TWO RViz windows
(one from the UR driver's default config, one from our MoveIt config).
The fix is `launch_rviz: 'false'` passed through
`onrobot1_ros/onrobot_description/launch/ur10e_rg6_control.launch.py`.

**Caveat:** `onrobot1_ros` is gitignored as a vendor package (bootstrapped
via `vcs import src < ros2.repos`). If you re-import vendor packages
the fix will be wiped and you'll see two RViz windows again until the
edit is re-applied. TODO: lift the `launch_rviz: false` arg-pass into
our own `full_stack.launch.py` so it survives vendor refresh.

## Detailed: `move_group.launch.py`

Just the move_group node. Use when you've already launched UR control
separately (or you're running against a different controller stack).

Loads (in this order) and merges into the move_group parameter set:
- `robot_description` from the combined xacro
- `robot_description_semantic` from `ur10e_rg6.srdf`
- `robot_description_kinematics` from `kinematics.yaml`
- `robot_description_planning` — **merged** from `joint_limits.yaml`
  + `pilz_cartesian_limits.yaml`. Loading them as separate dicts caused
  the second to overwrite the first (moveit2 issue #1691) → Pilz couldn't
  find `rg6_joint`'s velocity/acceleration limits and threw `map::at`.
- `ompl_planning.yaml` + `pilz_planning.yaml` for the two pipelines

### Args

| Arg | Default | Notes |
|---|---|---|
| `use_fake_hardware` | `true` | Passed into the xacro to switch hardware plugin |
| `robot_ip` | `127.0.0.1` | Passed into the xacro |

## Detailed: `moveit_rviz.launch.py`

Just RViz with the MoveIt MotionPlanning panel, loaded with
`config/moveit.rviz`. Reads the same `robot_description` /
`robot_description_semantic` / `robot_description_kinematics` as
move_group so the planning groups + collision matrix are available.

Use when iterating on the `moveit.rviz` view config — kill `rviz2`,
re-run this without restarting the rest of the stack:

```bash
pkill -9 -f rviz2
ros2 launch ur10e_rg6_moveit_config moveit_rviz.launch.py
```

### Args

| Arg | Default | Notes |
|---|---|---|
| `use_fake_hardware` | `true` | |

## Detailed: `demo.launch.py`

Stock MoveIt demo configuration with mock controllers. Bypasses
`ur_robot_driver` entirely. Useful for pure planning experiments where
you don't need controller-level realism.

```bash
ros2 launch ur10e_rg6_moveit_config demo.launch.py
```

## Detailed: vendor launches

**`ur10e_rg6_control.launch.py`** (in `onrobot_description`): wraps
`ur_robot_driver`'s `ur_control.launch.py` with our combined URDF
(`ur10e_rg6.urdf.xacro`) and the dual-RViz fix (`launch_rviz: 'false'`).
Args forwarded: `ur_type=ur10e`, `robot_ip`, `use_fake_hardware`,
`description_file`. Always included by `full_stack.launch.py`; calling
directly is rare.

**`view_ur10e_rg6.launch.py`**: URDF + joint_state_publisher_gui + a
standalone RViz with `view_robot.rviz`. No controllers, no MoveIt.
Use when something looks geometrically wrong and you want to scrub
the URDF without controller noise.

**`test.launch.py` / `test_gazebo.launch.py`**: vendor scratch. Gazebo
isn't part of our setup; treat as not-for-use.

## Killing things cleanly

The launches are tied together by the `ros2 launch` parent process. To
restart the whole stack:

```bash
pkill -9 -f 'ros2 launch'
pkill -9 -f move_group
pkill -9 -f rviz2
pkill -9 -f ros2_control_node
pkill -9 -f robot_state_publisher
pkill -9 -f spawner
# wait ~3 s
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
```

To restart only RViz (keep controllers + move_group alive):

```bash
pkill -9 -f rviz2
ros2 launch ur10e_rg6_moveit_config moveit_rviz.launch.py
```

To restart only move_group (e.g. to pick up a new SRDF):

```bash
pkill -9 -f move_group
ros2 launch ur10e_rg6_moveit_config move_group.launch.py
```

## Verifying everything is up

`tests/check_real_hw_network.sh` doubles as a stack-health probe (run
it with `192.168.1.100` for real hw or `127.0.0.1` for sim). Quick
manual checks:

```bash
# Should print "active" rows for joint_state_broadcaster,
# scaled_joint_trajectory_controller, rg6_gripper_controller
ros2 control list_controllers

# Should list /move_action
ros2 action list | grep move_action

# Should publish all 7 joints at 500 Hz
ros2 topic hz /joint_states
```

## Related

- [WSL2 ↔ UR10e networking](../docs/WSL2_UR10e_NETWORKING.md) — pendant
  prereqs + WSL2 setup that must be done BEFORE real-hw launch
- [Real-hardware connection](real_hw_connection.md) — ports + URCaps
  + topics the launches actually use
- `tests/check_real_hw_network.sh` — pre-flight script that confirms
  the cabinet is reachable before launch
- `tests/measure_real_robot_pose.py` — read-only post-launch verification
  that the kinematic model agrees with the real cabinet

## Last updated

2026-05-24.

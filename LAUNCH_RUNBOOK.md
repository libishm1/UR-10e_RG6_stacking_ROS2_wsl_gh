# Manual Launch Runbook

The exact, working sequence to bring up the UR10e + RG6 + MoveIt stack from a
cold WSL shell. Use this if `full_stack.launch.py` ever misbehaves or if you
want to launch each piece in a separate terminal so you can read logs.

## 0. Hard reset — kill any leftover processes from previous sessions

Stale RViz / move_group instances from a crashed terminal are the #1 source of
"robot model not loading" or duplicate-node weirdness. Always start clean.

```bash
# Open WSL
wsl -d Ubuntu-22.04

# Kill anything ROS-related from a previous run
ps -ef | grep -E 'ros2|rviz2|move_group|ros2_control_node' | grep -v grep \
  | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 3
# Verify nothing left
ps -ef | grep -E 'ros2|rviz' | grep -v grep | wc -l   # should print 0
```

## 1. Source the environment (every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/ur_rg6_ws/install/setup.bash
```

## 2. Easiest path — one-shot launch

```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
```

Wait ~20 s. Brings up: UR controllers + RSP + RViz + (after 10 s timer) gripper
JTC spawner + (after 12 s) move_group + (after 15 s) MoveIt RViz panel.

If you only see ONE RViz window: you're good. If two appear, kill all and
re-run step 0.

## 3. Step-by-step (when full_stack misbehaves)

### 3a. UR control stack (terminal 1)

```bash
ros2 launch onrobot_description ur10e_rg6_control.launch.py use_fake_hardware:=true
```

Wait until `scaled_joint_trajectory_controller` shows as `active`:

```bash
# In a separate terminal — same env sourced
ros2 control list_controllers | grep scaled
```

### 3b. Gripper JTC (terminal 2 — only when controller_manager is up)

This is the one that historically had a race:

```bash
ros2 run controller_manager spawner rg6_gripper_controller \
  -t joint_trajectory_controller/JointTrajectoryController \
  -p ~/ur_rg6_ws/install/ur10e_rg6_moveit_config/share/ur10e_rg6_moveit_config/config/rg6_jtc.yaml \
  --controller-manager-timeout 10
```

**Note** the `-t` (--controller-type) flag — without it the spawner asks the
controller_manager for the type as a param, and there's a race where the param
isn't set yet. Pass it directly with `-t` and the race is bypassed.

Verify:
```bash
ros2 control list_controllers | grep rg6_gripper_controller   # → active
ros2 action list | grep rg6_gripper_controller                 # → follow_joint_trajectory
```

### 3c. MoveIt move_group (terminal 3)

```bash
ros2 launch ur10e_rg6_moveit_config move_group.launch.py use_fake_hardware:=true
```

Wait for `You can start planning now!` in the log.

### 3d. RViz (terminal 4)

**Make sure no other RViz is running first** (Section 0). Otherwise you get two
windows fighting over the same `/robot_description`.

```bash
ros2 launch ur10e_rg6_moveit_config moveit_rviz.launch.py use_fake_hardware:=true
```

## 4. Verify everything works

```bash
# All three planning groups plan + execute
python3 ~/ur_rg6_ws/tests/test_groups.py

# Pilz PTP on arm_with_gripper, 20 varied goals
python3 ~/ur_rg6_ws/tests/test_pilz_hammer.py

# Bridge endpoints (topics, actions, services)
python3 ~/ur_rg6_ws/tests/test_bridge_endpoints.py

# Visible arm + gripper demo in RViz
python3 ~/ur_rg6_ws/tests/demo_full_safe.py
```

All should report PASS / SUCCESS.

## 5. Stop everything

```bash
# Ctrl+C in each terminal first, then sweep
ps -ef | grep -E 'ros2|rviz2|move_group|ros2_control_node' | grep -v grep \
  | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 3
```

Or for the nuclear option (also resets WSL networking):
```bash
# From PowerShell, NOT WSL
wsl --shutdown
```

## Common symptoms → fixes

| Symptom | Cause | Fix |
|---|---|---|
| "Robot model not loading" / TF errors / red links in RViz | Two RViz instances running, or move_group not up | `ps -ef \| grep rviz2` — if more than one, kill them all and restart |
| Gripper controller "Failed loading controller" | Spawner race on `type` param | Pass `-t joint_trajectory_controller/JointTrajectoryController` explicitly |
| "Action client not connected to action server: rg6_gripper_controller/..." | Gripper JTC didn't spawn | Run section 3b manually |
| Pilz fails with `No ContextLoader for planner_id ''` | RViz Context tab algorithm dropdown is empty | Set it to PTP/LIN/CIRC |
| Pilz LIN fails with `No solver for group arm_with_gripper` | Union group has no IK | Use `ur_manipulator` group for LIN/CIRC |
| Pilz fails with `map::at` | `joint_limits.yaml` and `pilz_cartesian_limits.yaml` not merged | Already fixed in `move_group.launch.py` — rebuild if stale |
| `urscript_interface` spamming "Failed to connect to robot on 127.0.0.1:30002" | Harmless in fake-hardware mode | Ignore |

## Reference paths

- Workspace: `~/ur_rg6_ws/`
- Source code: `~/ur_rg6_ws/src/ur10e_rg6_moveit_config/`
- Installed configs: `~/ur_rg6_ws/install/ur10e_rg6_moveit_config/share/ur10e_rg6_moveit_config/config/`
- This file: `~/ur_rg6_ws/LAUNCH_RUNBOOK.md`
- Session state and project history: `~/ur_rg6_ws/SESSION_HANDOFF.md`

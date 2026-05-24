# UR-10e_RG6_stacking_ROS2_wsl_gh

UR10e + OnRobot RG6 pick-and-place stack on ROS 2 Humble + MoveIt 2, running
in WSL2 (Ubuntu 22.04) on Windows. Plays an 80-waypoint URScript program in
simulation with visualised box stacking.

## Quick start

```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
python3 tests/play_pickplace.py
```

See `SESSION_HANDOFF.md` for the full project state, design notes, and
real-hardware path. See `LAUNCH_RUNBOOK.md` for manual launch / debug steps.

## Bootstrapping vendor packages

The vendor packages under `src/` (`moveit2`, `Universal_Robots_*`, `ur_msgs`,
`ur_client_library`, `onrobot1_ros`, `moveit_resources`) are excluded from
this repo via `.gitignore`. They are pinned to exact commits in
[`ros2.repos`](ros2.repos) and bootstrapped with `vcs import`:

```bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
cd ~/ur_rg6_ws
vcs import src < ros2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

To track upstream HEADs instead of the pinned SHAs, change each
`version:` field in `ros2.repos` from a commit hash to a branch name
(`humble` for the UR + MoveIt packages, `ros2` for `onrobot1_ros` and
`moveit_resources`, `master` for `ur_client_library`), then re-run
`vcs import src < ros2.repos --force`.

The combined URDF (`ur10e_rg6.urdf.xacro`) and SRDF live in this repo
and overlay the upstream UR description after import.

## Packages in this repo

- `src/ur_description/` — UR10e description.
- `src/ur10e_rg6_moveit_config/` — combined UR10e + RG6 MoveIt 2 config,
  SRDF, kinematics, joint limits, controllers, RViz layout, launch files.
- `tests/` — verification + demo scripts (Pilz coverage, OMPL coverage,
  `play_pickplace.py`, `gripper_test.py`, calibrators).
- `docker/`, `grasshopper/` — Windows-side bridging.

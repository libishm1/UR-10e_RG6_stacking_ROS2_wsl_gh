# UR-10e_RG6_stacking_ROS2_wsl_gh

UR10e + OnRobot RG6 pick-and-place stack on ROS 2 Humble + MoveIt 2, running
in WSL2 (Ubuntu 22.04) on Windows. Plays an 80-waypoint URScript program in
simulation with visualised box stacking.

<img width="800" height="600" alt="ros_resized" src="https://github.com/user-attachments/assets/e9eb7a84-4d89-4c94-bcba-aea8f4069cce" />

## Quick start

```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py
python3 tests/play_pickplace.py
```

See `SESSION_HANDOFF.md` for the full project state, design notes, and
real-hardware path. See `LAUNCH_RUNBOOK.md` for manual launch / debug steps.
For WSL2 ↔ UR10e networking (fallback ladder + diagnostic recipes), see
[`docs/WSL2_UR10e_NETWORKING.md`](docs/WSL2_UR10e_NETWORKING.md).

```bash
# Real-hardware pre-flight (run before `ros2 launch ... use_fake_hardware:=false`)
~/ur_rg6_ws/tests/check_real_hw_network.sh 192.168.1.100

# Minimal smoke test (dry-run first, then --yes to actually move)
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py                       # dry-run
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes                 # sim arm + sim gripper
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --real-gripper  # real arm + real gripper
```

## Docker (no ROS install needed)

A self-contained Docker image is provided that clones this repo, imports the
vendor packages via [`ros2.repos`](ros2.repos), and pre-builds the workspace —
all you need is Docker Desktop + WSL2:

```bash
# Build the self-contained image (~15-20 min, ~3 GB)
docker compose -f docker/docker-compose.yml --profile build_only build ur10e_rg6_full

# Launch the full MoveIt 2 + UR driver + RViz stack (fake hardware)
docker compose -f docker/docker-compose.yml up full_stack

# Run the pick-and-place demo against the running container
docker exec -it ur10e_rg6_full python3 /workspace/tests/play_pickplace.py
```

See [`docker/README.md`](docker/README.md) for the full Docker workflow,
including the dev image (workspace mounted from host) and the real-hardware
path with `ROBOT_IP`.

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

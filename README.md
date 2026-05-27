# UR-10e_RG6_stacking_ROS2_wsl_gh

UR10e + OnRobot RG6 pick-and-place stack on ROS 2 Humble + MoveIt 2, running
in WSL2 (Ubuntu 22.04) on Windows. Plays an 80-waypoint URScript program in
simulation with visualised box stacking.

<img width="800" height="600" alt="ros_resized" src="https://github.com/user-attachments/assets/e9eb7a84-4d89-4c94-bcba-aea8f4069cce" />

## Quick start (operator scripts)

The supported entry point is the `scripts/` folder. Each script is
idempotent, kills leftover ROS first to avoid duplicate RViz, and
prints what to do next.

```bash
# Simulation (RViz only — no hardware required)
bash scripts/launch_sim.sh                       # bring up MoveIt + ros2_control mock
bash scripts/play_pickplace.sh --max 4           # 2 full pick+place cycles
bash scripts/kill_ros.sh                         # tear down

# Real hardware (cabinet at 192.168.1.100; see HARDWARE_TEST_HANDOFF.md
# for pendant prereqs the first time you run on a fresh setup)
bash scripts/launch_real.sh                      # ping + RTDE probe + launch
# … load external_control.urp on pendant, press Play, switch to Remote Control …
bash scripts/play_pickplace.sh --max 4 --real-gripper   # 2 cycles with real RG6
bash scripts/kill_ros.sh
```

See [`scripts/README.md`](scripts/README.md) for the full operator
reference (flows, gotchas, flag list). For a step-by-step
hardware-test runbook (with acceptance criteria at each step), see
[`HARDWARE_TEST_HANDOFF.md`](HARDWARE_TEST_HANDOFF.md).

Background reading:
- [`SESSION_HANDOFF.md`](SESSION_HANDOFF.md) — full project state, design
  decisions, calibration values.
- [`LAUNCH_RUNBOOK.md`](LAUNCH_RUNBOOK.md) — manual launch sequence for when
  the one-shot scripts misbehave.
- [`docs/WSL2_UR10e_NETWORKING.md`](docs/WSL2_UR10e_NETWORKING.md) — WSL2 ↔
  cabinet networking fallback ladder + diagnostics.
- [`wiki/`](wiki/) — locked decisions, known bugs, mechanism comparisons.

Low-level smoke tests (kept for debug, but `scripts/` is preferred for
day-to-day use):

```bash
# Real-hardware network pre-flight only
~/ur_rg6_ws/tests/check_real_hw_network.sh 192.168.1.100

# Single-motion smoke test (dry-run first, then --yes to actually move)
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py                       # dry-run
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes                 # sim arm + sim gripper
python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --real-gripper  # real arm + real gripper

# Gripper alone (binary close/open via UR Tool I/O on pin 16)
python3 ~/ur_rg6_ws/tests/onrobot_io_grip.py cycle
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

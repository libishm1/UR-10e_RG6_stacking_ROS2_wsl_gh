# UR-10e_RG6_stacking_ROS2_wsl_gh

UR10e + OnRobot RG6 pick-and-place stack on ROS 2 Humble + MoveIt 2, running
in WSL2 (Ubuntu 22.04) on Windows. Plays an 80-waypoint URScript program in
simulation with visualised box stacking.

<img width="800" height="600" alt="ros_resized" src="https://github.com/user-attachments/assets/e9eb7a84-4d89-4c94-bcba-aea8f4069cce" />

## Command cheat-sheet (operator quick reference)

Run everything from the workspace root in a **normal (non-admin) WSL terminal**.
The `scripts/*.sh` launchers kill leftover ROS/socat first (no duplicate RViz),
so you rarely call `kill_ros.sh` yourself.

### Simulation (no hardware)
```bash
cd ~/ur_rg6_ws
bash scripts/launch_sim.sh                 # MoveIt + mock hardware + RViz
# wait ~10 s for controllers to go active (else the first move CONTROL_FAILEDs)
bash scripts/play_pickplace.sh --max 4     # 2 pick-place cycles (sim gripper)
bash scripts/view_rviz.sh                  # (optional) single attached RViz window
bash scripts/kill_ros.sh                   # stop everything
```

### Real hardware — full pick-place (arm + RG6 gripper)
```bash
cd ~/ur_rg6_ws
bash scripts/launch_real_rs485.sh          # arm stack + tool-RS485 gripper bridge
#  On the pendant: load the `ros` installation, then Play external_control.urp
#  in Remote Control mode. REQUIRED — the gripper bridge runs in that program.
bash scripts/play_pickplace.sh --max 4 --real-gripper
bash scripts/kill_ros.sh
```

### Real hardware — arm only (no gripper)
```bash
cd ~/ur_rg6_ws
bash scripts/launch_real.sh                # ping + RTDE probe + launch
#  Pendant: Play external_control.urp, Remote Control mode
bash scripts/play_pickplace.sh --max 4
bash scripts/kill_ros.sh
```

### Gripper alone — open / close / status (no arm, no Claude)
Needs the stack up via `launch_real_rs485.sh` **with `external_control.urp` playing**:
```bash
cd ~/ur_rg6_ws
bash scripts/grip.sh open            # open ~150 mm
bash scripts/grip.sh close           # close / grip object at 40 N
bash scripts/grip.sh close 30 60     # close to 30 mm at 60 N
bash scripts/grip.sh status          # width + grip_detected
bash scripts/grip.sh cycle           # close then open
```
An object is "gripped" when the jaws stop short of the commanded width.

### Stop · build · recover
```bash
bash scripts/kill_ros.sh             # hard-stop all ROS + socat (idempotent)
colcon build --symlink-install       # only after C++/xacro changes
```
- **RViz is a tiny/blank stub, or the gripper Modbus returns nothing** → WSLg or
  the cabinet's rs485 daemon is wedged (often after `wsl --shutdown` or an
  **admin-mode** WSL session). Fix: `wsl --shutdown` in **PowerShell**, reopen a
  **non-admin** WSL terminal; if the gripper still won't talk, **restart
  PolyScope** and reload the `ros` installation.

More: [`scripts/README.md`](scripts/README.md) (flows, flags) ·
[`wiki/rg6_rs485_modbus.md`](wiki/rg6_rs485_modbus.md) (gripper/RS485 setup) ·
[`wiki/known_bugs_and_workarounds.md`](wiki/known_bugs_and_workarounds.md) (gotchas) ·
[`HARDWARE_TEST_HANDOFF.md`](HARDWARE_TEST_HANDOFF.md) (step-by-step hw runbook).

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

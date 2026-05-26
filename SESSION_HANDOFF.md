# UR10e + RG6 — Session Handoff

Last updated: 2026-05-23. Read this first; it covers the current state and how
to pick up where we left off.

## CHECKPOINT — 2026-05-26 (late afternoon — SHOULDER-PAN SIGN FIX FOUND, real-HW deployment caveat documented)

**🎯 Visual orientation FIXED in sim.** After more iteration, the root cause
turned out to be a **shoulder_pan sign mismatch** between URDF and real
cabinet: same joint value produces opposite physical direction.

> **✅ VERIFIED ON REAL HARDWARE 2026-05-26 late-afternoon.**
> ur_rtde readback with real robot at physical HOME confirms cabinet
> uses `shoulder_pan = +π/2` while our sim uses `−π/2` for the same
> visual pose. **Δ = 180° on shoulder_pan, 0 on all other joints.**
> The URDF axis convention is sign-inverted from this cabinet.
>
> **Deployment caveat:** scripts use `-π/2` for sim correctness.
> Sending `-π/2` to the real cabinet → arm rotates to wrong side.
> Use one of three deployment strategies (re-teach cabinet HOME,
> sign-flip at driver boundary, or fix URDF axis) before Phase 5+
> real-hw motion. Details in
> [`wiki/shoulder_pan_sign_mismatch.md`](wiki/shoulder_pan_sign_mismatch.md).

### The fix

Change `shoulder_pan_joint` from `+1.5708` to `-1.5708` in HOME:
- `tests/play_pickplace.py` HOME_Q
- `tests/real_hw_smoke.py` HOME_Q
- `config/ur10e_rg6.srdf` `home` group_state
- `config/initial_positions.yaml`

Sim pickplace 10/10 with the new value, RViz visual now matches the
physical cell at HOME. **Locked.**

Full details: [`wiki/shoulder_pan_sign_mismatch.md`](wiki/shoulder_pan_sign_mismatch.md).

### Critical caveat for real hardware deployment

**Sim and real cabinet now use OPPOSITE shoulder_pan values for the
same physical HOME pose.** Operator-known HOME on the real cabinet
uses `+pi/2`; our scripts use `-pi/2` for sim correctness. Sending
our `-pi/2` to the real robot will rotate the arm AWAY from the work
area (the OPPOSITE side from real HOME).

**Three options for Phase 5+ real-hardware deployment:**

1. **Re-teach the cabinet HOME** to `[-pi/2, -pi/2, -pi/2, -pi/2, pi/2, pi/2]`
   — operator adopts new convention, scripts work in both sim and real
2. **Sign-flip at the driver boundary** — keep `-pi/2` in scripts for
   sim, programmatically negate shoulder_pan before sending to the
   real driver. Adds complexity but preserves visualization.
3. **Investigate / fix the URDF axis convention** — proper but deeper
   work; would need digging into shoulder_pan_joint axis in `ur_macro.xacro`

Decision: TBD. Document in [`shoulder_pan_sign_mismatch.md`](wiki/shoulder_pan_sign_mismatch.md).

### New documentation

- [`wiki/shoulder_pan_sign_mismatch.md`](wiki/shoulder_pan_sign_mismatch.md)
  — the fix, the caveat, the deployment options
- [`wiki/known_bugs_and_workarounds.md`](wiki/known_bugs_and_workarounds.md)
  — consolidated catalog of all session bugs + workarounds:
  - Shoulder-pan sign mismatch (this one)
  - Bare URScript on 30002 crashing URCap
  - WSLg pink-window after many launches
  - Dual RViz spawn
  - mock_components initial_positions parsing warning
  - WSL2 NAT vs UR reverse interface
  - OnRobot URCap cold-boot quirk
  - pickplace LIN→PTP retry noise
  - Calibration extraction doesn't fix the 1m+ TCP-Z mismatch

### Phase 5 prerequisites (NEW — must verify before real-hw motion)

Per `D:\robot_ws\reference\deep-research-wsl2_networking.md`:

1. **Confirm WSL2 mirrored mode** is active: `cat ~/.wslconfig` shows
   `networkingMode=Mirrored` ✅ (current state)
2. **Set Windows firewall** for inbound + outbound TCP 50001-50003 from
   the cabinet's IP (192.168.1.100). One-time PowerShell admin command.
3. **Verify `reverse_ip` parameter** in the UR driver is the Windows
   host LAN IP (192.168.1.35), NOT auto-detected. **Likely TODO.**
4. **Set `keepalive_count` / `robot_receive_timeout`** in the driver
   YAML (default may drop connections too aggressively).
5. **Decide shoulder_pan deployment strategy** (per the 3 options above)
   before sending any motion command to the real robot.

## CHECKPOINT — 2026-05-26 (afternoon — RViz visual mismatch closed as cosmetic)

User at the cell. After extensive attempts to make RViz visually match the
physical robot orientation at HOME (multiple URDF rotations, mesh visual
overrides, calibration applications), **none fixed the visual mismatch.**
Decision: accept it as cosmetic-only and move forward.

### Full attempt list (everything we tried, none fixed it)

1. Apply `kinematics_parameters_file` from `ur_calibration calibration_correction`
   → small per-link corrections, no gross orientation change. Calibration is
   still in URDF (harmless, may help later).
2. Rotate `base_link_inertia` visual mesh by `${pi/2}` (was `${pi}`).
   Reverted: base mesh is rotationally symmetric so the rotation was
   invisible.
3. Apply `<origin xyz="0 0 0" rpy="0 0 pi">` to the `ur_robot` macro mount.
   Reverted: rotated the whole scene 180° in world, but the arm in RViz
   STILL appears flipped relative to the cabinet body — same as before
   from the user's comparison angle.
4. Various RViz camera angle changes (Yaw 0, π, 5π/4 etc.). Cosmetic only,
   doesn't fix.
5. `wsl --shutdown` reset of WSLg. Fixed a separate "pink window" rendering
   issue but didn't change the kinematic-visual.

### Conclusion / decision

**Accept the visual mismatch as a known cosmetic limitation.** Reasons:
- `tests/measure_real_robot_pose.py` showed sim TCP at HOME has Z ≈ 1.485 m
  but real TCP Z ≈ 0.400 m — a 1m+ kinematic-model mismatch the per-link
  calibration doesn't fix. This is the root cause; rotation just shuffles
  symptoms.
- Real-hardware motion will work correctly because the controller uses
  its own factory calibration regardless of what our URDF predicts.
- RViz visualization is still useful for planning + collision checking,
  just not pixel-accurate.

### Documented locations

- `wiki/rviz_visual_orientation_mismatch.md` — full investigation,
  attempt list, dive-deeper options if future session wants visual parity
- Memory: `project_ur10e_rg6_workspace.md` 2026-05-26-afternoon entry +
  lesson learned ("don't grind on URDF rotations to fix visual-only issues
  when kinematics work")
- This handoff (above checkpoint)

### Current state (committed and on GitHub)

- URDF: reverted to default mount (`rpy="0 0 0"`); mesh rotation back
  to nominal `${pi}`; calibration yaml `ur10e_cell_calibration.yaml` is
  loaded via `kinematics_parameters_file`
- All `play_pickplace.py` 10/10 verified in sim with current state
- Single RViz window, no dual-RViz, WSLg working after `wsl --shutdown`

### Next: Phase 5 of validation plan

`python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --no-gripper`
on the real cell. Slow (5%), small joint perturbation (±0.05 rad), hard
caps. First actual real-arm motion.

## CHECKPOINT — 2026-05-26 (at-the-cell session)

**User physically at the cell, real UR10e + RG6 powered on.** Worked through
Phases 0-4 of the validation plan; deferred Phase 5+ pending the visual
mesh fix.

### What worked

- **Phase 0-2 (pre-flight + pendant + network).** All pre-existing — the
  cell was set up from prior `D:\robot_ws` sessions. `check_real_hw_network.sh`
  reports 7/0 pass against `192.168.1.100`.
- **Phase 4 (read-only kinematic verification).** Installed `ur_rtde` via
  `pip3 install --user ur_rtde`. Ran `measure_real_robot_pose.py` against
  the real cell at HOME:
  - Joint values match `HOME_Q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]`
    to within 0.00° — HOME is correct.
  - TCP-in-base-frame: real = `(0.176, 0.691, 0.400)`, sim (default
    kinematics) = `(0.001, 0.532, 1.485)` — large discrepancy.
- **Calibration extraction.** Ran
  `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.100`
  successfully. Yaml written to
  `src/ur10e_rg6_moveit_config/config/ur10e_cell_calibration.yaml`,
  values match the calibrated DH from `D:\robot_ws\reference\dodectest3.urp`.
  URDF `ur10e_rg6.urdf.xacro` now points at this yaml via
  `kinematics_parameters_file`. Pushed as commit `db5c0c5`.
- **Sim pickplace 10/10 with calibration applied.** Two LIN→PTP auto-retries
  mid-run as before; no regression from calibration.

### What didn't work / unresolved

- **Visual mismatch in RViz vs real cell:** at HOME, real-robot's gripper
  hangs over the cable side (manufacturer's `-X` of base_link); our URDF
  visualization shows the gripper over `+X` (manufacturer's front). User
  said: "poses look correct, table is correct, only the robot base itself
  looks rotated." So **kinematics are functionally correct** (URScript
  poses, IK, planning all working), but the **base cabinet mesh is
  visually misoriented** relative to where the physical cabinet sits.
  Cosmetic only — doesn't affect motion. **Open question for next step.**
- **Direct URScript on port 30002 + `rg_grip()` CRASHED the URCap.**
  Without the URCap preamble (only loaded when a `.urp` is playing),
  bare URScript on the secondary client interface caused the cabinet
  to error out and require restart. Documented as a "known issues"
  entry in `wiki/real_hw_connection.md`. Future Claude sessions should
  NEVER reach for this — use Path B or External-Control-Play instead.
- **WSLg window state corruption.** After many launch/relaunch cycles
  today, RViz windows stopped rendering ("pink window" stuck state).
  Fix: `wsl --shutdown` from PowerShell, then reopen WSL terminal.
  Resets all WSLg Qt/OpenGL state cleanly. After the reset, RViz
  worked normally again.

### Next session steps (still open)

1. Apply visual-only base cabinet mesh fix (cosmetic) — keep kinematics
   intact, just rotate the cabinet visualization to match the physical
   cell orientation.
2. Phase 5: `real_hw_smoke.py --yes --no-gripper` for the first actual
   real-hardware arm motion at 5% speed.
3. Phase 6: gripper smoke test via Path B (proven by the URCap crash
   incident that bare URScript on 30002 is NOT a viable gripper path).
4. Phases 7-9.

## CHECKPOINT — 2026-05-25 (real-hardware validation prep)

**Status:** sim baseline locked. All artifacts needed to walk the cell
up to verified pick-place on real hardware are now in place. The next
session at the cell uses the validation plan below — no more sim
experiments needed.

**New artifact:** [`wiki/real_hw_validation_plan.md`](wiki/real_hw_validation_plan.md) —
**9-phase step-by-step checklist** with pass/abort criteria at each
step and a fill-in validation log at the bottom. Designed so one
operator at the cell can run it end-to-end with a hand on the E-stop.
Phase summary:

- Phase 0 — Pre-flight (off-robot, WSL + Windows firewall sanity)
- Phase 1 — Pendant prereqs (URCap installs + Remote Control + Network)
- Phase 2 — Network reachability (ping + TCP probes + Dashboard handshake)
- Phase 3 — Driver bring-up (External Control handshake, NO motion)
- Phase 4 — Read-only kinematic verification (`measure_real_robot_pose.py`)
- Phase 5 — Arm-only smoke (`real_hw_smoke.py --yes --no-gripper`, ±3 cm @ 5%)
- Phase 6 — Gripper-only smoke (URCap path via `gripper_test.py --real`)
- Phase 7 — Combined arm + gripper smoke (`real_hw_smoke.py --yes --real-gripper`)
- Phase 8 — Single pick-place cycle (`play_pickplace.py --real-gripper --max 1 --force 25`)
- Phase 9 — Full 10-cycle program (`--force 40`, normal speed)

**Hard rule:** don't advance past any failing phase. Diagnose first.
Rollback procedure is in the plan.

**Open visual-orientation question** — STILL UNRESOLVED, but no longer
a blocker: the validation plan's Phase 4.3 will measure the actual
yaw mismatch between URDF mount and real cabinet (visual observation
+ optional `rpy` correction). Once we have that one number, we set it
in the URDF and the question is closed.

**Files supporting the plan:**
- `tests/check_real_hw_network.sh` — Phase 2 automation
- `tests/measure_real_robot_pose.py` — Phase 4 automation (read-only)
- `tests/real_hw_smoke.py` — Phases 5 + 7 automation (slow, hard-capped)
- `tests/gripper_test.py` — Phase 6 automation
- `tests/play_pickplace.py` — Phases 8 + 9 automation

**Memory entries supporting the plan:**
- [[reference-ur10e-cell-network]] — verified IPs + SSH key paths
- [[reference-path-b-deploy]] — fallback URScript deploy if ROS path fails
- [[feedback-motion-speeds]] — always default to slow
- [[feedback-rviz-ghost-intent]] — ghost on for manual, untick for scripts
- [[feedback-wiki-habit]] — promote durable findings to wiki

## CHECKPOINT — 2026-05-24 (visual orientation experiments — REVERTED to defaults)

**Status:** Visual orientation mismatch between ROS RViz and the real cell
is **UNRESOLVED**. User reports the real robot has its arm hanging OVER the
cable side at HOME (cable exits the manufacturer's "back" = -X in base_link),
while our ROS RViz with default URDF shows the arm at +X (manufacturer's
"front"). User also reports Grasshopper does not match either of those.

**What we tried:**
- URDF `<origin rpy="0 0 3.14159">` (180° yaw on the UR mount). Pick-place
  passed 10/10 each time (kinematics + base_link-anchored boxes work fine),
  but user reports it still didn't visually match the real cell or Grasshopper.
- RViz camera angle changes (Yaw=π, then Yaw=5π/4). Didn't help either.

**Current state (all reverted to clean defaults):**
- URDF `<origin xyz="0 0 0" rpy="0 0 0">` — default mount
- moveit.rviz Views back to simple `Distance: 2.0, Focal Point: (0,0,0.6)`
- VEL_SCALE = 0.08 in play_pickplace.py, ACC_SCALE = 0.08 — slow/safe for real hw
- VEL_SCALE = 0.05 in real_hw_smoke.py — even slower for first-ever real-hw test

**What we KEPT (good improvements regardless of orientation):**
- Boxes + pedestal in `play_pickplace.py` are `BASE_LINK`-anchored, NOT `world`.
  Lets future base-rotation experiments work without box drift.
- Dual-RViz fix: `onrobot1_ros/...ur10e_rg6_control.launch.py` passes
  `launch_rviz: 'false'` (caveat: vendor dir, gets wiped on `vcs import`).
- HOME = `[1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]` everywhere.

**New: `tests/measure_real_robot_pose.py` — pure read-only RTDE measurement.**
Connects to the real UR10e on port 30004 via `ur_rtde.RTDEReceiveInterface`,
samples joint angles + TCP pose for ~1 s, prints a comparison vs our ROS
HOME constant. **Never writes to the robot** — RTDEControlInterface /
URScript / Dashboard are all gated out (import check on entry).
- Use to verify the robot is at HOME and that our kinematic model agrees
  (TCP-in-base-frame matches between ROS URDF and the real cabinet).
- Will NOT solve the URDF-vs-room yaw mismatch from RTDE data alone —
  that requires visual observation of the cabinet's physical orientation,
  documented in the script's Verdict section.
- Loopback hard-refused unless `--allow-loopback` so URSim can't be
  mistaken for the real cell.
- Install: `pip3 install --user ur_rtde`.

**Next step is real-hardware verification, not more guessing in sim.**
Per user 2026-05-24: bring up the real cell at slow speeds (real_hw_smoke.py
first with `--no-gripper`, then `--real-gripper --force 25 --max 1`). When
the arm physically moves you can compare its real orientation to what RViz
shows at the same joint values, and the orientation mismatch becomes a
single concrete number (yaw degrees).

## CHECKPOINT — 2026-05-24 (later, RViz visual cleanup)

- **Dual-RViz bug fixed.** `onrobot1_ros/onrobot_description/launch/ur10e_rg6_control.launch.py`
  now passes `launch_rviz: 'false'` to the included `ur_control.launch.py`. Previously
  every `full_stack` launch spawned TWO RViz windows: one from the UR driver's default
  `view_robot.rviz` and one from our `moveit_rviz.launch.py`. Verified single RViz
  process after the fix.
  **CAVEAT:** this edit lives in a gitignored vendor package (`src/onrobot1_ros/`).
  If you re-run `vcs import src < ros2.repos`, the fix is wiped. TODO: lift the fix
  into our own `full_stack.launch.py` so it survives a vendor re-import; for now
  the change must be re-applied after every vendor refresh.
- **Collision objects re-anchored from `world` to `base_link`.** All three
  `co.header.frame_id = "world"` lines in `tests/play_pickplace.py` (box spawn,
  box detach, pedestal) changed to `BASE_LINK`. This lets us rotate the URDF base
  later WITHOUT the boxes drifting relative to the robot. Verified 10/10 pickplace
  still works.
- **URDF base rotation experiment.** Set `rpy="0 0 3.14159"` on the ur_robot mount,
  rebuilt, ran pickplace 10/10 — kinematics fine. But the visual orientation STILL
  didn't match the user's Grasshopper view. Reverted to `rpy="0 0 0"`. The visual
  mismatch turned out to be RViz camera angle (looking from -X) vs Grasshopper
  Perspective viewport (looking from -X-Y).
- **RViz camera matched to Grasshopper Perspective.** `moveit.rviz` updated:
  `Yaw=3.927` (5π/4) + `Pitch=0.6` + `Focal Point=(0.4, 0, 0.4)` + `Distance=2.5`.
  This places the camera at roughly (-X, -Y, +Z) looking back at origin — same
  viewpoint as Rhino's default Perspective. The robot, table, and arm pose then
  appear the same in both visualisations.
- **RViz ghost (`Query Goal State`) — locked decision.** Default `true` (on) for
  manual interactive control. RViz has no runtime API to toggle from a script, so
  the user manually unticks the checkbox in the MotionPlanning panel before long
  scripted runs. See persistent memory `feedback_rviz_ghost_intent.md`. Don't
  oscillate the default — it's stable as ON.

## CHECKPOINT — 2026-05-24 (real-hardware bring-up prep)

Added the real-hardware path end-to-end:

- [`docs/WSL2_UR10e_NETWORKING.md`](docs/WSL2_UR10e_NETWORKING.md) — deep
  dive on WSL2 ↔ UR10e with a four-level fallback ladder
  (mirrored → bridged → NAT+portproxy → native Linux), diagnostic recipes
  for the canonical failures, and a user-only-tasks table.
- [`tests/check_real_hw_network.sh`](tests/check_real_hw_network.sh) —
  pre-flight diagnostic: ICMP + TCP probes on all 5 UR ports + Dashboard
  handshake + reverse-channel listener check. Run before every real-HW
  launch.
- [`tests/real_hw_smoke.py`](tests/real_hw_smoke.py) — minimal arm Z up/down
  + gripper cycle around HOME at 5% speed. Joint perturbation hard-capped
  at 0.10 rad (≈ 6 cm TCP). Dry-run by default; `--yes` to execute.
  `--no-gripper` skips gripper steps entirely (then ONLY needs
  ur_robot_driver + any MoveIt-for-UR — no RG6 ROS boilerplate).
  `--real-gripper` uses URScript topic, which goes through the
  OnRobot URCap on the pendant — also needs no RG6 ROS config.
  Use this as the FIRST motion test on real hardware — arm-only first,
  then add `--real-gripper` for the URCap path.
- [`wiki/`](wiki/) — durable findings ([index](wiki/index.md),
  [real_hw_connection](wiki/real_hw_connection.md),
  [path_b_vs_ros_driver](wiki/path_b_vs_ros_driver.md)).
  Pattern borrowed from `D:\robot_ws\robots\wiki\`. Habit going
  forward: durable research → wiki page (not chat).
- [`src/ur10e_rg6_moveit_config/launch/full_stack.launch.py`](src/ur10e_rg6_moveit_config/launch/full_stack.launch.py)
  bug fix — `robot_ip` arg is now forwarded to the onrobot child launch
  (was silently defaulting to `127.0.0.1` for real-hardware launches).
- Verified cell config saved here (Networking section below) sourced
  from `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md`:
  laptop 192.168.1.35, cabinet 192.168.1.100, SSH key at
  `D:\robot_ws\robots\outputs\2026-05-09\ssh_setup\robots_workspace_key`
  already enrolled on the pendant.
- Test verification (against the still-running fake-HW stack):
  `test_groups.py` PASS on all 3 groups, `gripper_test.py` clean,
  `play_pickplace.py` 10/10 earlier today.

GitHub: pushed to https://github.com/libishm1/UR-10e_RG6_stacking_ROS2_wsl_gh
with `ros2.repos` (pinned vendor packages), `docker/Dockerfile.full`
(self-contained image), and the networking docs above.

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

## Networking (real hardware)

`~/.wslconfig` on Windows host has `networkingMode=mirrored` enabled. This is
the recommended way to reach a physical UR10e on the LAN from WSL2.
After any edit to `.wslconfig`, run `wsl --shutdown` and reopen — see
`docker/NETWORKING.md` for the full guide.

### Verified cell config (from `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md`)

| Item | Value |
|---|---|
| Laptop static IP | `192.168.1.35` |
| UR10e cabinet IP | `192.168.1.100` |
| Subnet | `255.255.255.0` |
| Gateway | (empty — direct ethernet) |
| Cabinet MAC | `00:30:D6:41:1C:13` |
| Polyscope | `5.24.0.1219432` |
| Robot s/n | `20255201551` |
| URCaps | OnRobot (RG6 driver) |

### SSH key (already enrolled on the pendant)

The private/public pair lives at `D:\robot_ws\robots\outputs\2026-05-09\ssh_setup\`:
- Private: `robots_workspace_key` (keep on laptop, never share)
- Public: `robots_workspace_key.pub` (already installed on pendant as `robots-workspace-2026-05-10`)

Reuse this key — do not generate a new one. Pendant import is documented in
`D:\robot_ws\robots\outputs\2026-05-09\ssh_setup\usb_payload\README_USB_IMPORT.txt`.

SSH user is `root`. Quick smoke test from WSL once mirrored networking is up:
```bash
KEY="/mnt/d/robot_ws/robots/outputs/2026-05-09/ssh_setup/robots_workspace_key"
chmod 600 "$KEY"
ssh -i "$KEY" -o StrictHostKeyChecking=accept-new root@192.168.1.100 'ls /programs'
```

### Pendant-side prerequisites (must do once on the UR)

1. Settings → Security → **enable all 5 services** (29999 / 30001 / 30002 /
   30003 / 30004) — they ship DISABLED. Without this, ALL TCP ports
   timeout and the ROS 2 driver can't connect.
2. Settings → Security → General → change "Disable inbound access to
   additional interfaces (by port)" from `1-65535` to `1-21,23-65535`
   (excludes port 22 so SSH works).
3. Settings → Security → Secure Shell → enable, authentication "Both".
4. Top-right toggle: **Remote Control mode**.
5. Install the **External Control URCap** and create a program that
   contains a single `external_control` node configured for Host IP =
   `192.168.1.35` (laptop), Port `50002`.

### Launch with real hardware

```bash
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
    use_fake_hardware:=false \
    robot_ip:=192.168.1.100
```

Wait ~20 s for everything to come up, then **press Play on the pendant**
on the External Control program. The terminal logs
`Robot connected to reverse interface` when handshake completes.

OnRobot URCap cold-boot quirk: first Play after a cold cabinet boot
triggers "RG grip didn't initialize" and the cabinet shuts down. Restart
the cabinet, then Play again — second attempt picks up the URCap. This
is REPEATABLE.

Start with `play_pickplace.py --real-gripper --force 25 --max 1`, hand on
E-stop, no people in the work envelope.

### Path B fallback (URScript deploy via SFTP + Dashboard)

If ROS 2 streaming isn't right for a particular task (e.g. need to run an
operator-authored .urp that uses URCap functions), the reference workflow
is at `D:\robot_ws\robots\outputs\2026-05-10\path_b\urp_deploy.py`. It
takes a URScript body, splices it into a template .urp's `<cachedContents>`,
retargets the `<file>` ref, uploads via SFTP, and dashboard-loads+plays.
Verified V3 on 2026-05-10.

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

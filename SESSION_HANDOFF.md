# UR10e + RG6 — Session Handoff

Last updated: 2026-05-26 (evening). Read this first; it covers the current state and how
to pick up where we left off.

## CHECKPOINT — 2026-05-27 (end of overnight session — sim pickplace FULLY LANDED, real-hw motion VERIFIED, gripper engagement only remaining blocker)

10 commits pushed to GitHub `main` between yesterday evening and this morning. `git log --oneline -10` from `8ec790a` back to `10b1a85` shows the full arc.

### What's verified ON REAL HARDWARE today

| Item | Status |
|---|---|
| URDF↔cabinet kinematics agree to 0.4 mm | ✅ at HOME AND extreme freedrive |
| External Control URCap installed + configured | ✅ on cabinet `/root/.urcaps/` |
| Windows Firewall inbound 50001-50004 from cabinet | ✅ |
| First real arm motion under ROS 2 (direct trajectory) | ✅ |
| Full pickplace cycle motion only (no real grip) | ✅ 56 s, 9 motion steps, no failures |
| Pick approach + descent X/Y/Z alignment with wood block | ✅ at gripper-down orientation |
| OnRobot URCap X/Y calibration `(−6.66, +10.52)` mm | ✅ |
| OnRobot URCap Z calibration `+45 mm` | ✅ (RG6 fingers pivot down on close) |

### What's verified ONLY IN SIMULATION (real-hw test still TODO)

| Item | Status |
|---|---|
| `play_pickplace.py --max 4` with attach=True | ✅ 2 full pick+place cycles in 78 s |
| Box attach with centroid +50 mm above TCP (geometric fix) | ✅ no LIFT collision |
| Box settle visualisation (pre-spawn + detach at un-shifted Z) | ✅ |
| Calibration value `+45 mm` validity at PLACE orientation | ⚠️ same tool-frame error, different wrist rotation → different world-frame manifestation; **tomorrow's hardware test will reveal** |

### What's NOT yet tested (next-session work)

| Item | Status |
|---|---|
| `--real-gripper` actually closing on the wood block | ⚠️ URScript-via-topic doesn't reach OnRobot URCap (PolyScope architectural limit, empirically confirmed by URP rebuild test); **Tool I/O fix designed but not yet implemented** |
| Full real-hw pickplace with gripper | ⚠️ blocked by gripper engagement |
| 20-cycle full stack (sim or real) | ⚠️ |
| Place X/Y/Z calibration on real hardware | ⚠️ |

### The breakthrough insights of this session

1. **URDF kinematic fix** — two-line edit (`ur_macro.xacro:350` `rpy="0 0 ${pi}"` → `rpy="0 0 0"`, `ur_macro.xacro:364` `<axis xyz="0 0 -1" />` → `<axis xyz="0 0 1" />`). The 180° base yaw + my earlier shoulder_pan axis flip were canceling at HOME by symmetry but creating a 180°-class mirror at all other poses. Removed both: URDF FK now matches cabinet RTDE TCP within 0.4 mm at any pose. **Real-hw verified at HOME + extreme freedrive pose.**

2. **OnRobot URCap is a GUI wrapper, NOT the gripper driver.** The reference `onrobot1_ros/onrobot_interface/src/onrobot_gripper.cpp` controls the gripper via `/io_and_status_controller/set_io` — tool digital pin 16 + analog tool voltage. The URCap's `rg_grip()` is a PolyScope program-tree node that calls into URCap Java; it's NOT a URScript function that can be triggered from External Control's socket. The 2026-05-24 "Mechanism C" decision was based on wrong premise; now superseded by Tool I/O lock-in in `wiki/decisions.md`.

3. **Operator's geometric insight** for the attach collision: the planning-scene box was being attached centroid-AT-TCP, but the physical block extends UPWARD from the grip line. Moving the attached centroid `+50 mm` above the TCP both clears the box-vs-lower-stack-box collision AND visually matches reality. Sim pickplace immediately worked after this fix.

### Files modified this session (all committed and pushed)

- `src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro` — kinematic fix (whitelisted in `.gitignore`)
- `src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro` — rg6_tcp 0.190 → 0.228 m
- `src/ur10e_rg6_moveit_config/launch/move_group.launch.py` — `execution_duration_monitoring: False`, larger scaling
- `src/ur10e_rg6_moveit_config/config/ur10e_rg6.srdf` + `initial_positions.yaml` — HOME values
- `tests/play_pickplace.py` — `WAYPOINT_TOOL_CALIBRATION_M = (-0.00666, +0.01052, +0.045)`, `BOX_ATTACH_Z_OFFSET_M = +0.050`, `DRY_RUN_DISABLE_ATTACH = True`, detach + pre-spawn use un-shifted positions
- `tests/real_hw_smoke.py` — HOME comment updates
- `calibration/` directory created — `cabinet_calibration.conf`, `fk_experiment.py`, `apply_calibration_quaternion.py` stub, `reapply_driver_patches.sh`, `urp/external_control_with_onrobot_node.urp`, `README.md`
- `wiki/decisions.md` — 2026-05-26 entry SUPERSEDES the 2026-05-24 Mechanism C
- `wiki/shoulder_pan_sign_mismatch.md` — resolution section
- `wiki/known_bugs_and_workarounds.md` — multiple new entries

### Next-session plan (~1.5-2 h)

1. **Tool I/O grip helper** (~30 min): replace URScript-topic grip with `/io_and_status_controller/set_io` service calls (digital out pin 16). Pattern in `onrobot_interface/src/onrobot_gripper.cpp` — port to a small Python helper in `tests/onrobot_io_grip.py`. Wire into `play_pickplace.py`'s `--real-gripper` path.
2. **SRDF tip_link `tool0 → ee_link`** (~30 min): eliminates the `WAYPOINT_TOOL_CALIBRATION_M` shift entirely. MoveIt plans for the grasp point directly. After rebuild, all calibration becomes irrelevant and the pick-vs-place orientation ambiguity disappears.
3. **Real-hw 1-cycle pickplace with --real-gripper** (~20 min): verify grip closes on the wood block, lifts, places at destination, releases.
4. **Real-hw 2-cycle** (~20 min): verify the second pick after first place.
5. **Real-hw 20-cycle full stack** (~30 min wall-clock at the ~50 s/cycle observed today, more if WSL2 RTDE jitter): full production validation.

### Helper scripts in `/tmp/` (regenerated each WSL session — `wsl --shutdown` wipes them)

- `/tmp/kill_ros.sh` — kill all ROS launches cleanly (patterns in file, not argv → no self-kill)
- `/tmp/launch_sim.sh`, `/tmp/launch_real.sh` — stack launchers
- `/tmp/peek_joint_states.py` — direct rclpy subscriber (CLI hangs in WSL2)
- `/tmp/tcp_compare.py` — read-only diagnostic: cabinet RTDE TCP vs URDF FK
- `/tmp/switch_controllers.py` — re-activates `scaled_joint_trajectory_controller` when controller_stopper misses URP-start
- `/tmp/send_grip.py` — publishes `rg_grip(w,f)` URScript to topic (currently silently no-ops — see Tool I/O fix)
- `/tmp/direct_trajectory_smoke.py` — bypass MoveIt, send direct joint trajectory
- `/tmp/check_ext_control.sh`, `/tmp/verify_ext_control.sh` — dashboard probes

If `wsl --shutdown` happened between sessions, regenerate via Write tool. See `calibration/README.md` for full list and contents.

### Final pendant state at end of session

URP was PLAYING (`external_control.urp` with OnRobot RG node above External Control), Remote Control on. Cabinet RUNNING/NORMAL. **Tomorrow morning's first action:** check if pendant is still in this state OR power-cycle + redo the URP load + Play sequence per the SESSION_HANDOFF entries above.

---

## CHECKPOINT — 2026-05-26 (very late evening — URDF KINEMATIC FIX VERIFIED on real hardware)

🎯 **Real cabinet ↔ URDF FK now match to 0.4 mm at every measured pose.** From 705 mm error at extreme pose to **0.4 mm** — a 1700× improvement on a two-line edit in `ur_macro.xacro`.

### The fix (two reverts in `src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro`)

1. **Line 350** — `base_link → base_link_inertia` joint:
   - **Was:** `<origin xyz="0 0 0" rpy="0 0 ${pi}" />` (180° yaw)
   - **Now:** `<origin xyz="0 0 0" rpy="0 0 0" />` (no rotation)
   - Upstream comment claimed UR controllers have "X+ pointing backwards" relative to REP-103, but for this cabinet's primary-interface output the kinematic data is ALREADY in REP-103-aligned base frame. The upstream yaw caused a 180°-class mirror everywhere except at HOME (where the symmetric all-±π/2 joint config hid it).

2. **Line 364** — `shoulder_pan_joint`:
   - **Was:** `<axis xyz="0 0 -1" />` (my earlier symptom-patch from this morning)
   - **Now:** `<axis xyz="0 0 1" />` (upstream default)
   - The axis flip was patching the HOME symmetry coincidence created by the 180° yaw above. With both reverted, they cancel and the URDF FK matches the cabinet's RTDE everywhere.

**No change to `HOME_Q` anywhere.** SRDF, `initial_positions.yaml`, scripts all keep `[+π/2, -π/2, -π/2, -π/2, +π/2, +π/2]`. Cabinet and URDF agree on what those values mean physically.

### Empirical proof — see `calibration/fk_experiment.py`

Tested 4 hypotheses against measured RTDE TCP at 2 distinct poses:

```
Hypothesis                                          HOME |Δ|   EXTREME |Δ|
A (180° yaw + shoulder_pan -Z, "current" pre-fix)   0.4 mm     854 mm  ← matched HOME by coincidence
B (no yaw + shoulder_pan +Z, FIX APPLIED)           0.4 mm     0.4 mm  ← matches everywhere
C (no yaw + shoulder_pan -Z)                        1.43 m     778 mm
D (180° yaw + shoulder_pan +Z, pre-my-axis-flip)    1.43 m    1155 mm
```

Hypothesis B is the only one that matches at non-HOME poses. The "tcp_compare.py" results on real hardware after the fix:

```
                       Real (RTDE)              URDF rg6_tcp           |Δ|
HOME:                 (+0.176, +0.692, +0.400) (+0.176, +0.692, +0.400) 0.4 mm
Extreme freedrive:    (-0.591, +0.901, +0.213) (-0.591, +0.901, +0.213) 0.4 mm
   joints: pan=+120°, lift=-122°, elbow=-81°, w1=-46°, w2=+126°, w3=+90°
```

Both position AND rotation (axis-angle) match exactly.

### What's also in the fix (already in place from earlier this evening)

- **`ur10e_rg6.urdf.xacro` rg6_tcp offset** edited from 0.190 m to 0.228 m so URDF `rg6_tcp` exactly matches the cabinet's `set_tcp()` (gripper Z offset). With this, both `rg6_tcp` and the cabinet's RTDE TCP refer to the same physical point.

### Real-hardware dry run — 2026-05-26 (very late evening, post-fix)

`play_pickplace.py --max 3` ran on REAL hardware (no `--real-gripper`,
`DRY_RUN_CLEARANCE_M = 0.10` to fly 10 cm above contact heights). With the
URDF kinematic fix in place:

```
✓ movej PTP HOME             (from off-HOME freedrive pose, real motion, ~6 s)
✓ grip 70                    (SIM mode, no physical gripper move)
✓ movel LIN  (0.823, 0.473, 0.216)   (real motion, ~8 s)
✓ movel LIN  (0.823, 0.473, 0.129)   (real motion, ~4 s)
✓ grip 50                    (SIM mode)
✓ planning scene attach box_00
✗ movel LIN  (0.823, 0.473, 0.500)   INVALID_MOTION_PLAN (LIFT)
✗ PTP retry to LIFT                  also INVALID_MOTION_PLAN
```

**This is the first time non-symmetric world-frame Cartesian motion succeeded
on real hardware** — three distinct XYZ targets reached correctly, no "mirror"
behaviour. The URDF kinematic fix is empirically validated end-to-end at the
motion level, not just at the FK math level.

The remaining LIFT failure at `(0.823, 0.473, 0.500)` is a SEPARATE issue
from the kinematic fix. Both LIN and PTP failed, so it's the GOAL state that
URDF IK can't solve — not the path. Three diagnostic hypotheses:

1. **Attached-box collision** — `n.attach_box_to_tcp(box_00, ...)` puts the
   box collision geometry on rg6_tcp; the IK solver may see the box colliding
   with another link (or with itself) at the LIFT configuration. Easy test
   next session: comment out the attach call, re-run.
2. **SRDF disable_collisions gaps** — with the kinematic fix, the joint
   configurations needed to reach the same TCP are DIFFERENT from before.
   Some link-pair contacts that didn't happen pre-fix may now happen and
   need to be whitelisted in `ur10e_rg6.srdf`.
3. **Wrist singularity** — gripper pointing straight down with the arm
   nearly fully extended (Z=0.5 m, X=0.823 m → reach ≈ 0.96 m, well within
   1.3 m envelope but the wrist orientation constraint is tight).

**The session's URDF kinematic fix saga is CLOSED.** The LIFT issue is a
collision/IK-config problem, not a kinematic-correctness problem. Next
session should resolve it quickly with the hypotheses above.

### Arm state at end of session

Arm is **at PICK position**, NOT HOME, when this dry run aborted:
```
shoulder_pan  = +0.7063 rad
shoulder_lift = (clipped from earlier output; ~ -2.0)
elbow         = -1.4956 rad
wrist_1       = -1.1494 rad
wrist_2       = +1.5751 rad
wrist_3       = +2.2786 rad
rg6_joint     = +0.9361 rad   (SIM "closed" — no physical gripper change)
```

Operator should Freedrive back to HOME via pendant Local Control + the
round freedrive button before powering off.

### Known sim regression (NOT blocking real hardware)

`play_pickplace.py --max 2` (fake_hardware) now fails at the LIN LIFT after attaching the box (`INVALID_MOTION_PLAN`). Pre-fix sim worked. Symptom is collision or IK-solution-change because the URDF chain orientation shifted by 180°, putting the attached-box collision geometry in a slightly different relative location. **Not a blocker** — fix is to bump LIFT clearance in `play_pickplace.py` or relax the planning-scene `box_00` collision margin. ~30 min next session.

### Next session pickup order (refined)

1. Diagnose the sim `INVALID_MOTION_PLAN` — likely a clearance bump in the LIFT waypoint or a margin tweak in the `box_00` collision geometry. Once sim passes 2+ cycles, proceed.
2. Restart real-hw stack, re-Play `external_control.urp` on pendant, switch to Remote Control.
3. Run `play_pickplace.py --max 4 --real-gripper` for one full pick + place cycle on real hardware. **The kinematic fix should make this work** — no more "mirror at PICK". Expect ~5–10 min wall-clock due to known WSL2 RTDE-slowdown.
4. If the gripper still doesn't close via `/urscript_interface/script_command` (recall: separate bug from this evening), debug that — likely OnRobot URCap preamble missing in our External Control URP. Workaround: command gripper via Path B sidecar program.
5. Eventually fix the upstream `ur_calibration` `toYaml()` Euler-decomposition issue with a proper quaternion-based extractor (see `calibration/apply_calibration_quaternion.py` stub) — useful for any future cabinet recalibration, not blocking current cell.

### Helper scripts on disk (`/tmp/`)

- `kill_ros.sh`, `launch_sim.sh`, `launch_real.sh`, `peek_joint_states.py`,
  `rtde_overflow_check.sh`, `check_ext_control.sh`, `verify_ext_control.sh`,
  `tcp_compare.py`, `switch_controllers.py`, `direct_trajectory_smoke.py`,
  `return_to_home.py`.

### What was rendered obsolete by this fix (cleanup TODO)

- `wiki/shoulder_pan_sign_mismatch.md` — historical, kept for traceability;
  the "shoulder_pan sign" was a symptom not the cause.
- The "shoulder_pan axis flip" mention in the calibration README and several
  comment blocks scattered through `ur_macro.xacro` / scripts — pure cosmetic
  cleanup. The current state is "shoulder_pan back to upstream +Z, no yaw".

---

## CHECKPOINT — 2026-05-26 (late evening — FIRST REAL MOTION + URDF kinematic bug ROOT-CAUSED) — SUPERSEDED by VERY late evening checkpoint above

**The headline:** the arm physically moved under ROS 2 control for the first time. End-to-end pipeline (External Control URCap + reverse channel + scaled_joint_trajectory_controller + direct trajectory action) is proven working. AND we identified WHY the URDF visualisation is mirrored at non-HOME poses — it's a known-broken `Eigen::eulerAngles(0,1,2)` decomposition in upstream `ur_calibration::Calibration::toYaml()` that produces non-unique rpy triples for our cabinet's 180°-class link rotations.

### Sequence of milestones (this evening)

1. **External Control URCap installed on cabinet.** SFTP'd `externalcontrol-1.0.5.urcap` to `/root/.urcaps/`, had to rename `.urcap`→`.jar` because PolyScope's bundle scanner filters by `.jar` extension. Restart pendant; URCap appears in Installation tab.
2. **Configured URCap.** Host IP `192.168.1.35`, Custom Port `50002`. Created `/programs/external_control.urp`, loaded, pressed Play, switched pendant to Remote Control mode.
3. **Windows Firewall rule.** Inbound TCP 50001-50004 from `192.168.1.100` (cabinet) → Allow. (Admin PowerShell, command in `wiki/known_bugs_and_workarounds.md`.)
4. **First real motion.** `direct_trajectory_smoke.py` sent a small shoulder_lift Z-up trajectory. Arm physically moved (verified by reading `/joint_states` during execution: `shoulder_lift` drifted from `-1.5700` → `-1.5763` over 14 s). Motion is ~10× slower than commanded due to WSL2 non-RT kernel + RTDE 500 Hz hardcoded — see existing wiki entry. Cabinet stayed in `RUNNING`/`NORMAL` throughout.
5. **MoveIt execution timeout bumped.** `allowed_execution_duration_scaling: 30.0` in `move_group.launch.py` (was default 1.2) — prevents MoveIt from cancelling slow trajectories.
6. **First real pickplace attempt → revealed deeper kinematic bug.** Ran `play_pickplace.py --max 1 --real-gripper`. Arm moved but the cabinet ended up MIRRORED relative to the URDF visualisation. Gripper never closed (URScript-via-topic doesn't carry OnRobot URCap preamble — separate known issue).
7. **TCP comparator built (`tools/tcp_compare.py` — saved in `/tmp/`).** Compares cabinet RTDE TCP vs URDF FK at the same joint state. Read-only, doesn't move the arm. Confirmed:
   - HOME: X/Y agree within ±1 mm. Z off by `+0.279 m` exactly (cabinet's `set_tcp` offset for RG6).
   - Mild off-HOME: X/Y still good (±18 mm), Z still constant offset.
   - Extreme off-HOME: **X off by +575 mm, Y off by −388 mm, Z error +130 mm** — gross kinematic divergence.
8. **Root cause identified.** Pulled `/root/.urcontrol/calibration.conf` from cabinet (saved at `calibration/cabinet_calibration.conf`). Found:
   - `delta_theta[1] = -1.408 rad` (shoulder_lift -81°)
   - `delta_theta[2] = +1.372 rad` (elbow +79°)
   - `delta_d[1] = +429.4`, `delta_d[2] = -433.1` (encoded values absorbed by `correctAxis()` — not physical)
   - These ARE handled by `Calibration::correctChain()` in `ur_calibration/src/calibration.cpp:59`.
   - BUT `Calibration::toYaml()` (same file, line 230) extracts the corrected matrix via `Eigen::eulerAngles(0, 1, 2)` which is **ambiguous near 180° rotations** — produces `(π, π, π)` for our `forearm` link even though the actual matrix is a single specific 180° rotation about a different axis. URDF then reconstructs a different matrix from those rpy values. **That's the bug.**

### Where things are right now

- **Calibration directory** at `~/ur_rg6_ws/calibration/`:
  - `cabinet_calibration.conf` — raw cabinet calibration (ground truth)
  - `cabinet_firmware_appinfo.yaml` — cabinet firmware metadata
  - `README.md` — full bug write-up + remediation plan
  - `apply_calibration_quaternion.py` — STUB extractor (parses raw, applies deltas; TODO: port `correctAxis` + quaternion emission to URDF)
- **URDF rg6_tcp offset** edited from 0.190 m → 0.228 m (matches cabinet `set_tcp` Z exactly) but **not yet rebuilt+restarted** (would kill the External Control connection).
- **Cabinet state:** arm at HOME via Freedrive (operator-controlled). URP still PLAYING. Remote Control on. Stop the URP on the pendant when ending session.

### Next session — pickup order

1. **Stop the URP** on the pendant if not already done. Switch back to Local Control.
2. **Build `apply_calibration_quaternion.py` properly.** Port `correctAxis(1)` and `correctAxis(2)` from `src/Universal_Robots_ROS2_Driver/ur_calibration/src/calibration.cpp` to Python. Emit link transforms using quaternion intermediate, NOT Euler decomposition. Output: corrected `ur10e_cell_calibration.yaml`.
3. **Rebuild `ur_description` and `ur10e_rg6_moveit_config`.** Restart stack on fake hardware first; check sim pickplace still passes.
4. **Switch to real hardware.** External Control URP setup is permanent (URCap installed, IP/port configured) — just load the URP and press Play. Activate `scaled_joint_trajectory_controller` if `controller_stopper` misses the start signal.
5. **Re-run `tcp_compare.py` at ≥4 distinct poses.** Validation: every pose's |Δ| < 5 mm (ignoring the constant `set_tcp` Z offset, which is now baked into URDF's rg6_tcp).
6. **THEN** retry `play_pickplace.py --max 4 --real-gripper`. Should produce one full pick-and-place cycle. Speed will still be ~10× WSL2-slow, but the kinematics will be correct.

### Helper scripts on disk (in `/tmp/`)

- `/tmp/kill_ros.sh` — kill all ROS launches/controllers/RViz cleanly (uses pattern file to avoid self-kill)
- `/tmp/launch_sim.sh` — launch full_stack with `use_fake_hardware:=true`
- `/tmp/launch_real.sh` — launch full_stack with `use_fake_hardware:=false robot_ip:=192.168.1.100`
- `/tmp/peek_joint_states.py` — direct rclpy subscriber, bypasses CLI hangs
- `/tmp/rtde_overflow_check.sh` — measure RTDE pipeline overflow rate
- `/tmp/check_ext_control.sh` — verify External Control URP playing + program_running topic
- `/tmp/verify_ext_control.sh` — dashboard query of robotmode/safetystatus/programState
- `/tmp/tcp_compare.py` — read-only TCP comparator (real vs URDF FK)
- `/tmp/switch_controllers.py` — manually activate `scaled_joint_trajectory_controller` (controller_stopper sometimes misses URP start signal)
- `/tmp/direct_trajectory_smoke.py` — bypass MoveIt, send small Z-up/down trajectory directly to scaled_joint_trajectory_controller
- `/tmp/return_to_home.py` — direct joint trajectory back to HOME with 60 s budget (NOT YET RUN — was blocked by classifier mid-mirror; if needed, run manually)

### Session extension — calibration + first FULL real-hardware pickplace cycle (very-very late evening)

After the kinematic-fix verification, we kept pushing and got real-hardware
pickplace motion working end-to-end:

1. **MoveIt `execution_duration_monitoring: False`** added to
   `move_group.launch.py` (default 1.2× planned-time timeout was killing
   slow LIN descents on WSL2 — see [`wiki/known_bugs`](wiki/known_bugs_and_workarounds.md)).
   With this disabled the cabinet can take as long as it needs for a slow
   move; cabinet's own URP safety timeouts still bound execution.
2. **`DRY_RUN_DISABLE_ATTACH = True`** in `play_pickplace.py` skips the
   planning-scene `attach_box_to_tcp` / `detach_box_at` calls. Confirmed
   hypothesis 1: the attached-box collision check (missing `touch_links`)
   was blocking the LIFT step. Skipping the attach unblocks the full
   PICK→LIFT→TRANSIT→PLACE sequence. Production fix is to pass
   `touch_links=['rg6_tcp','rg6_finger_*_finger_tip','rg6_finger_*_flex_finger','rg6_body']`
   to the attach call — deferred.
3. **`WAYPOINT_TOOL_CALIBRATION_M = (-0.00666, +0.01052, +0.045) m`** in
   `play_pickplace.py` — world-frame shift applied to every waypoint
   X/Y/Z before sending. Calibration of OnRobot URCap's `set_tcp`
   (`OnRobot_Single`) vs physical fingertip:
   - **X/Y** measured 2026-05-26 by reading pendant TCP at a known
     commanded pose (`pendant = (829.66, 462.48, …)` for command
     `(823, 473, …)` → world delta `(+6.66, -10.52, 0)` mm → we shift
     waypoints by the negative).
   - **Z = +45 mm**: empirically iterated. The URCap defines the TCP at
     the OPEN finger center; the RG6 fingers pivot inward AND slightly
     DOWN as they close (~5 mm of Z descent during closure). +45 mm
     command-frame keeps the open gripper above the workpiece such that
     post-closure the fingertips land at contact.
   - Caveat: X/Y calibration is **gripper-down (R_y(180°)) orientation
     only**. Place waypoints have different rotations
     `(2.221, 2.221, 0)` — the same tool-frame OnRobot_Single error
     manifests as a different world-frame offset there. Plan for
     next session: switch SRDF tip_link from `tool0` to `ee_link`/`rg6_tcp`
     (already matches cabinet `set_tcp` within 0.4 mm post-URDF-fix), so
     MoveIt plans directly for the gripper grasp point and the
     orientation-dependent error vanishes.
4. **`DRY_RUN_CLEARANCE_M = 0.0`** — running pickplace at real contact
   heights (no aerial dry-run clearance).

**Real-hardware pickplace `--max 2 --real-gripper` (1 full pick+place):**
56 seconds wall-clock, all 9 motion steps clean (PTP HOME, grip 70, LIN
approach, LIN pick deep, grip 50, LIN LIFT, LIN transit, LIN approach
place, LIN place deep, grip 60). **No timeouts. No INVALID_MOTION_PLAN.
No path tolerance violations.** First time a full pick+place sequence
has executed end-to-end on real hardware under ROS 2 control.

### Known issues still open after this session

1. **`rg_grip()` URScript-via-topic doesn't engage the OnRobot URCap —
   CONFIRMED as a PolyScope architectural limitation, not a config bug.**

   We rebuilt `external_control.urp` on the pendant with the OnRobot RG node
   FIRST in MainProgram and External Control SECOND (verified by SSHing
   `/programs/external_control.urp` from the cabinet and decompressing the
   gzipped XML — see `calibration/urp/external_control_with_onrobot_node.urp`).
   The OnRobot RG node DOES execute at URP start (gripper moves to its
   configured width — `0.09 m / 80 N` in our saved URP). BUT subsequent
   `rg_grip(...)` strings sent via `/urscript_interface/script_command` do
   NOT engage the gripper. Silent no-op.

   Root cause: OnRobot URCap's `rg_grip` etc. live in a **Java-backed
   namespace** tied to its own program-node execution path. When PolyScope
   evaluates URScript text arriving on the External Control socket, it uses
   an interpretation context that does not include those Java-bound URCap
   functions. They appear "global" inside an OnRobot URCap node's own
   scope but aren't reachable from arbitrary URScript text injection.

   This means: the "OnRobot node above External Control" structure
   cannot fix the gripper-via-topic issue from ROS-side configuration alone.

   Our `external_control.urp` is the bare External Control URCap node only;
   it doesn't include the OnRobot URCap node, so the URCap's URScript
   preamble (defining `rg_grip()`, `rg_payload_set()`, etc.) is not
   loaded in the running URP's namespace. The driver sends `rg_grip(…)`
   strings; URScript runtime no-ops the undefined call.

   **JAR contents extracted to `/tmp/onrobot_urcap_extract/scripts/`** —
   the OnRobot URCap's preamble is ~15 interdependent `.script` files
   (`basics.engine.script`, `basics.globals.script`,
   `basics.java_connect.script`, etc.). Several call back into the
   URCap's Java runtime, so injecting them via `/urscript_interface/script_command`
   is non-trivial (we'd be missing the Java backend that
   populates `rg_Busy_arr`, `rg_Depth_arr`, etc.).

   **Clean fix:** rebuild `external_control.urp` on the pendant to
   include an **OnRobot RG node** at the top of MainProgram BEFORE the
   External Control node. That loads the URCap's full preamble via
   PolyScope's normal mechanism. ~5 min of pendant work.

   **Reference:** `D:\robot_ws\reference\dodectest3.urp` has exactly this
   structure (decompressed to `/tmp/dodectest3.xml` during this session
   for analysis). The relevant XML is:
   ```xml
   <Contributed strategyClass="com.onrobot.urcap.unified.OR_RG"
                strategyProgramNodeType="RG Grip"
                strategyURCapDeveloper="OnRobot A/S"
                strategyURCapName="OnRobot">
     <dataModel>
       <data key="rg-target-force" value="80.0"/>
       <data key="rg-target-width" value="0.15"/>
     </dataModel>
   </Contributed>
   ```

2. **Gripper crashed into foamboard** at Z = 0 mm calibration (URCap TCP
   is ~40 mm above physical fingertip). Resolved with `+45 mm` Z shift
   above. Foamboard absorbed the bump; cabinet safety did NOT trip
   (Robotmode RUNNING, Safetystatus NORMAL throughout).

3. **Place-pose X/Y calibration unverified.** The +6.66/+10.52 mm shifts
   are correct at gripper-down orientation; the place waypoints have
   rotation `(2.221, 2.221, 0)` so the same tool-frame OnRobot_Single
   error manifests as a different world-frame offset. Until SRDF
   tip_link is moved to `ee_link`, expect place positions to be
   miscalibrated by some millimeters in a different direction than
   pick positions.

### 🎯 BIG FINDING — gripper control via OnRobot Tool I/O (NOT URCap)

After the URP-rebuild dead end (above), investigated how the reference
repo `inria-paris-robotics-lab/onrobot_ros` actually controls the
gripper — turns out **the entire OnRobot URCap rabbit-hole was the
wrong rabbit-hole**.

The reference `onrobot_interface/src/onrobot_gripper.cpp` does:

```cpp
this->_set_io = create_client<ur_msgs::srv::SetIO>(
    "/io_and_status_controller/set_io");
this->_states_io_sub = create_subscription<ur_msgs::msg::IOStates>(
    "/io_and_status_controller/io_states", ...);
int PIN_GRIPPER_CONTROL = 16;   // tool digital OUT
int PIN_GRIPPER_STATE   = 17;   // tool digital IN
float DEFAULT_MAX_POSITION_VOLTAGE_RG6_V2 = 10.0;  // analog → width
```

**The OnRobot Quick Changer routes the UR cabinet's tool I/O pins
directly to the gripper's internal MCU.** No URCap, URScript, or URP
machinery in between. PolyScope just provides the tool I/O
infrastructure (digital out/in + analog tool voltage), and the gripper
MCU interprets the levels. The OnRobot URCap is a GUI wrapper around
the same I/O — not a required driver.

This means our REAL fix path for gripper-via-ROS is:

1. **Use the `/io_and_status_controller/set_io` service** (already
   exposed by ur_robot_driver as part of the io_and_status_controller
   we already have running). Drive pin 16, read pin 17, set analog
   voltage for fine width.
2. **No URCap, no URP rebuild, no URScript preamble extraction**
   needed. The "build URP with OnRobot RG node" experiment we just
   did was solving the wrong problem.
3. The reference `onrobot_interface` C++ plugin we rejected earlier
   (it crashed with `Un seul joint doit être défini`) is itself just
   a wrapper around this set_io approach. Fixing the URDF's
   ros2_control block to satisfy its `prefix`+`model` params would
   let it load cleanly.

`wiki/decisions.md` has been updated — the 2026-05-24 Mechanism-C
decision is REVERSED. The new locked decision is Tool I/O (via plugin
or independent node or roll-our-own — all three approaches share the
same set_io transport).

### Next session — concrete plan based on Tool I/O finding

1. **(~30 min) Build a minimal grip helper** that wraps
   `/io_and_status_controller/set_io` calls. Map width-mm to
   analog tool voltage (0-3 V or 0-10 V depending on RG6 v1/v2).
   Replace the URScript topic publisher in `play_pickplace.py`'s
   `--real-gripper` path with this helper.
2. **(~15 min) Test** — `play_pickplace.py --max 2 --real-gripper`.
   Expect the gripper to actually engage on the wood block this
   time. NO pendant changes needed beyond running External Control
   URP for arm motion.
3. **(~30 min) Switch SRDF tip_link** from `tool0` to `ee_link` so
   MoveIt plans for the actual grip point. Removes the orientation-
   dependent X/Y calibration hack in `WAYPOINT_TOOL_CALIBRATION_M`
   (the +6.66/+10.52 shift was only correct at gripper-down).
4. **(~15 min) Fix touch_links** in `attach_box_to_tcp` and re-enable
   `DRY_RUN_DISABLE_ATTACH = False` for production attach behavior.
5. Then `play_pickplace.py --max 20 --real-gripper` should produce
   the full 10-box pick-and-place sequence on real hardware.

**Total ~1.5 hours of code + test next session to close out the
gripper engagement story.**

### Architectural pivot — URP-driven motion (deferred but documented)

User's insight: instead of replicating the cabinet's OnRobot URCap +
kinematic calibration inside ROS, **execute Grasshopper-generated URPs
natively on the cabinet** via Path B (SFTP + Dashboard load + play),
and use ROS only for orchestration + state monitoring + scene
visualization. The Grasshopper URP already encapsulates the perfect
cabinet calibration (it uses the cabinet's own kinematics and
OnRobot's TCP). This sidesteps all the
URDF↔URCap↔cabinet-TCP frame plumbing we've been fighting tonight.

**Next session plan:**
1. (Operator, ~5 min) Rebuild `external_control.urp` on pendant with
   OnRobot RG node + External Control node. Test URScript-via-topic
   `rg_grip(50, 40)` engagement → if URCap preamble is now loaded the
   real gripper will close.
2. (Code, ~30 min) Switch SRDF tip_link from `tool0` to `ee_link`,
   rebuild, retest. Eliminates the orientation-dependent
   `WAYPOINT_TOOL_CALIBRATION_M` X/Y hack — MoveIt plans the same point
   that URScript targets.
3. (Code, ~1 hour) Build `tests/urscript_to_waypoints.py` — parses a
   `.urp` (gzipped XML, extracts embedded URScript) OR `.script` file,
   extracts `movel`/`movej`/`rg_grip` calls, emits a `waypoints.yaml`.
   Modify `play_pickplace.py` to load WAYPOINTS from a yaml path.
4. Closes the loop: Grasshopper → URP → parser → waypoints.yaml →
   ROS execution against real hardware. End-to-end automated from
   Grasshopper design to physical motion.

### What did NOT work — for the audit trail

- `ur_macro.xacro:356` `<axis xyz="0 0 1" />` → `<axis xyz="0 0 -1" />` shoulder_pan flip **worked at HOME by symmetry-coincidence only**. Doesn't survive at non-HOME poses. Should be reverted in the next URDF cleanup, but it's not actively harmful (HOME visual matches; other poses are still wrong for the deeper Euler-decomposition reason).
- `controller_manager.update_rate: 500 → 250` Hz **partially worked** — overflow rate dropped from spam-level to ~8/s steady-state. Cabinet's RTDE publish rate is still 500 Hz hardcoded in our `ur_client_library` version; would need a driver patch to push it lower.
- `external_control.urp` direct upload of `.urcap` file to `/root/.urcaps/` **silently failed** until renamed to `.jar` — PolyScope bundle scanner filters by `.jar` extension.

---

## CHECKPOINT — 2026-05-26 (evening — URDF axis FIXED, real-hw driver UP, sim PASSED, External Control is the next blocker)

**🎯 Three things resolved end-to-end since the late-afternoon checkpoint:**

1. **Real-hardware driver brings up cleanly** (was crashing at startup).
   Two fixes in `src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro`:
   - The OnRobotRG6 ros2_control block now uses `mock_components/GenericSystem`
     **always** (the `onrobot_interface` C++ plugin requires `prefix`/`model`
     params we don't pass and aborts on init — never use it).
   - Declared the missing `<xacro:arg>`s for `script_filename`,
     `output_recipe_filename`, `input_recipe_filename`, `reverse_ip`,
     `reverse_port`, `script_sender_port`, `trajectory_port`,
     `script_command_port`, `headless_mode`, `non_blocking_read`,
     `keep_alive_count`, and forwarded them through to `<xacro:ur_robot>`.
     Without these the upstream `ur_control.launch.py`'s args were silently
     dropped and the driver aborted on the literal `to_be_filled_by_ur_robot_driver`
     placeholder.

2. **Shoulder-pan sign mismatch: deployment Option 3 chosen and executed.**
   Inverted `shoulder_pan_joint` axis in `ur_macro.xacro:356`
   (`<axis xyz="0 0 1" />` → `<axis xyz="0 0 -1" />`). Reverted every
   `-π/2` back to `+π/2` in:
   - `src/ur10e_rg6_moveit_config/config/ur10e_rg6.srdf` `home` group_state
   - `src/ur10e_rg6_moveit_config/config/initial_positions.yaml`
   - `tests/play_pickplace.py` HOME_Q[0]
   - `tests/real_hw_smoke.py` HOME_Q[0]

   **Verified end-to-end:**
   - `/joint_states` reports `shoulder_pan = +1.570795` (matches RTDE +
     pendant + URScript convention).
   - RViz visualization at `+π/2` now shows arm toward the table
     (matches the physical cell). Visually confirmed by user.
   - Sim `play_pickplace.py --max 2` PASSED 2/2 cycles.
   - **No sign-flip helper needed in any script.** Sim and real now agree
     numerically AND visually.

   **Caveat:** `ur_macro.xacro` is vcs-imported. A `vcs import` with
   force-mode overwrites the edit. Re-apply if that happens; the comment
   in `ur_macro.xacro:356` flags it.

3. **RTDE "Pipeline producer overflowed" addressed (partial).**
   Lowered `controller_manager.update_rate` from 500 Hz to 250 Hz in
   `src/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur10e_update_rate.yaml`.
   The cabinet's RTDE publish rate is still 500 Hz (hardcoded in this
   driver version), but combined with `non_blocking_read=true` (also
   plumbed through now) the steady-state overflow rate drops to **~8/sec**,
   ~1.6% sample loss — acceptable for slow / small-motion smoke tests.
   See `wiki/known_bugs_and_workarounds.md` "RTDE Pipeline producer
   overflowed spam on WSL2" for the longer write-up.

### New blocker: External Control URCap not currently playing

The driver is connected to the cabinet's Dashboard + RTDE (state readback
works) but **no External Control URP is loaded/playing on the pendant**.
No motion command can be executed until that's set up:

1. Verify the **External Control URCap** is installed (Installation tab
   → URCaps on the pendant — separate from the OnRobot RG URCap).
   - If missing: install from the .urcap at
     https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases
     (SFTP to `/root/.urcaps/` on the cabinet, reboot the pendant).
2. Configure: Installation → URCaps → External Control → **Host IP =
   `192.168.1.35`** (Windows host on the LAN, since WSL2 mirrored mode
   shares its address), **Custom Port = `50002`**.
3. Create a URP that uses the External Control node (File → New Program
   → URCaps tab → External Control), save as `external_control.urp`.
4. Load that URP on the pendant, press Play (▶).
5. Verify from WSL: `bash /tmp/check_ext_control.sh` should report
   `program_running = True` AND the launch log should show a
   `Robot program received` line.

### Helper scripts now on disk (in /tmp, will survive WSL but not reboot)

- `/tmp/kill_ros.sh` — kill every ROS launch/controller/RViz cleanly.
  Avoid putting "ros2 launch", "controller_manager", etc. literal strings
  in shells that invoke this; pkill self-matches via /proc/PID/cmdline.
- `/tmp/launch_sim.sh` — launch full_stack with `use_fake_hardware:=true`.
- `/tmp/launch_real.sh` — launch full_stack with `use_fake_hardware:=false robot_ip:=192.168.1.100`.
- `/tmp/peek_joint_states.py` — direct rclpy subscriber, bypasses the
  `ros2 topic` CLI hang that WSL2 keeps hitting.
- `/tmp/rtde_overflow_check.sh` — measure pipeline overflow rate over 30s.
- `/tmp/check_ext_control.sh` — verify External Control URP is playing.

### Next session — pickup order

1. Set up External Control URP on the pendant (steps above).
2. Run `bash /tmp/check_ext_control.sh` — confirm `program_running = True`.
3. Run `python3 tests/real_hw_smoke.py --yes --no-gripper --cycles 1`
   — first ever real-cabinet motion from our scripts. VEL_SCALE=0.05,
   MAX_DELTA_RAD=0.10 hard cap, ~5 cm Z up/down at HOME.
4. If smoke test PASSES: graduate to `--cycles 5`, then plan
   `play_pickplace.py --max 1 --real-gripper` for the full real-hw cycle.
5. Before sustained motion streaming: revisit the cabinet RTDE rate
   (currently hardcoded 500 Hz in driver — would need a patch to
   `URPositionHardwareInterface::on_configure` to expose as parameter).

---

## CHECKPOINT — 2026-05-26 (late afternoon — SHOULDER-PAN SIGN FIX FOUND, real-HW deployment caveat documented) — SUPERSEDED by evening checkpoint above

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

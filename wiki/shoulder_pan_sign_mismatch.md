# Shoulder-pan joint sign mismatch between URDF and real cabinet

## Purpose

Record the finding that fixed our long-running RViz visual orientation issue:
our URDF's `shoulder_pan_joint` rotates the arm in the **opposite physical
direction** from this UR10e cabinet's controller at the same numerical joint
value. Flipping the sign of `shoulder_pan_joint` in HOME makes the SIM
visualization match the physical cell.

## What we observed (2026-05-26 at the cell)

At joint values `[1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]`:

- **Physical robot:** arm extends to one side of the cabinet (the side
  with the table) → gripper hangs over the table at HOME
- **Our URDF / RViz:** arm extends to the **opposite** side → gripper points
  away from the table

Other diagnostics:
- TF: `world → base_link` = identity (our URDF mount is `rpy="0 0 0"`)
- TF: `base_link → base_link_inertia` = 180° yaw (UR macro's built-in
  REP-103 vs UR-controller-frame conversion — comment in `ur_macro.xacro`
  explicitly notes this)
- TCP-in-base-frame: real `(0.176, 0.691, 0.400)` vs sim `(0.001, 0.532, 1.484)`
  — same +Y direction but ~1m Z mismatch (separate kinematic-model issue)

## What we tried that didn't fix it

| Attempt | Result |
|---|---|
| `<origin rpy="0 0 pi">` on `ur_robot` mount | Whole scene rotates 180° in world, but arm vs table relative orientation unchanged (because boxes are base_link-anchored, they rotate with the robot) |
| `<origin rpy="0 0 pi/2">` on base_link_inertia visual mesh | Cylindrical mesh is rotationally symmetric — no visible difference |
| `<origin rpy="0 0 -pi/2">` on mount | Same as `pi` rotation — scene rotates as one unit |
| Kinematic calibration extraction via `ur_calibration` | Small per-link corrections, doesn't fix gross arm direction |
| RViz camera angle changes | Cosmetic only |

## What DID fix it

**Flip the sign of `shoulder_pan_joint` in HOME_Q:**

```python
# Before (matches real cabinet's URScript HOME):
HOME_Q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]

# After (makes RViz visual match the physical cell at HOME):
HOME_Q = [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]
```

Updated in:
- `tests/play_pickplace.py` HOME_Q
- `tests/real_hw_smoke.py` HOME_Q
- `src/ur10e_rg6_moveit_config/config/ur10e_rg6.srdf` `home` group_state
- `src/ur10e_rg6_moveit_config/config/initial_positions.yaml`

Pickplace runs 10/10 cleanly with the new value.

## Why this works — root cause

The URDF's `shoulder_pan_joint` axis convention is opposite to the physical
cabinet's controller at this cell. Same numerical joint value produces
opposite physical rotation.

Possible explanations (uninvestigated):
1. URDF defines `<axis xyz="0 0 1">` on the joint; the real cabinet's
   firmware/calibration treats positive as the opposite direction
2. The `base_link → base_link_inertia` 180° rotation in `ur_macro.xacro`
   was intended to handle this but is being applied at the wrong layer
3. The specific cabinet's installation (PolyScope teach pendant) has the
   robot frame configured in a non-standard orientation

We didn't dig further because flipping the sign in our scripts is a
practical fix that unblocks visualization.

## ✅ VERIFIED ON REAL HARDWARE — 2026-05-26

**Status:** the URDF-vs-cabinet sign mismatch is now CONFIRMED with
ur_rtde readback from the physical robot at its HOME.

### Verification result

Real cabinet manually moved to its operator-known HOME via the
pendant. `tests/measure_real_robot_pose.py` reports:

```
Joint angles (rad)        |  ROS HOME (rad)         |  Δ (deg)
--------------------------+-------------------------+---------
shoulder_pan    +1.5708   |   +1.5708              |    -0.00
shoulder_lift   -1.5708   |   -1.5708              |    +0.00
elbow           -1.5708   |   -1.5708              |    +0.00
wrist_1         -1.5708   |   -1.5708              |    +0.00
wrist_2         +1.5708   |   +1.5708              |    -0.00
wrist_3         +1.5708   |   +1.5708              |    -0.00
```

Real cabinet at physical HOME → `shoulder_pan = +π/2`.
Our SIM scripts at the same visual pose → `shoulder_pan = −π/2`.
**Δ = 180° on shoulder_pan, 0 on all others. Verified.**

### ✅ DOUBLE-CONFIRMED via ur_robot_driver `/joint_states` — 2026-05-26 (late)

After bringing up the real-hardware stack with `use_fake_hardware:=false`,
a direct rclpy subscriber on `/joint_states` (cabinet at physical HOME):

```
shoulder_pan_joint   +1.570802
shoulder_lift_joint  -1.570781
elbow_joint          -1.570771
wrist_1_joint        -1.570786
wrist_2_joint        +1.570796
wrist_3_joint        +1.570824
rg6_joint            +0.770000   (mock_components initial_value, no real
                                  data — RG6 is mock-only on real hardware)
```

**The ur_robot_driver passes the RTDE value through unchanged — it does NOT
apply any sign correction between the cabinet and ROS.** This means:

- Whatever the RTDE pendant convention reports, `/joint_states` matches it.
- Our SRDF/initial_positions/scripts that use `shoulder_pan = −π/2` make
  the URDF visualize correctly **but would command the cabinet to the
  wrong side** if sent verbatim via `scaled_joint_trajectory_controller`.
- The deployment options in the "Critical caveat" section below are
  STILL THE OPEN DECISION before any motion command on real hardware.

### What this means

- Sim with `shoulder_pan = −π/2` produces the same physical-looking
  pose as real cabinet with `shoulder_pan = +π/2`
- Both produce TCP in roughly the same world position (over the table)
- The URDF's `shoulder_pan_joint` axis convention is sign-inverted
  from the real cabinet's controller
- This is a hardware/firmware-specific quirk of this cell (or a
  bug in upstream `ur_description` for this UR10e variant)



### What we believe (and need to confirm)

Hypothesis: URDF's `shoulder_pan_joint` and the cabinet's controller
use opposite sign conventions. If true:
- Real robot at `+pi/2` shoulder_pan → arm goes to side A (toward table)
- Sim at `-pi/2` shoulder_pan → arm goes to side A (matches real visual)
- Real robot at `-pi/2` shoulder_pan → arm goes to side B (away from table)

### Verification plan (do this BEFORE any motion command in Phase 5+)

1. **Pendant readback at HOME.** Manually move the real robot to its
   known HOME via the pendant. Read joint values displayed on the
   pendant. Confirm shoulder_pan reads `+90°` (per `dodectest3.urp`).
2. **`measure_real_robot_pose.py` with manual HOME.** With the cabinet
   at its physical HOME, run the script and compare reported joint
   values to our scripts' HOME_Q. The signs should DISAGREE on
   shoulder_pan if our hypothesis is correct.
3. **Tiny test motion (10° increment).** From real HOME, send a
   `movej` command with `shoulder_pan_target = current + 0.17` (about
   10°). Observe which direction the arm physically rotates. Compare
   to what RViz would predict at the same target.
4. **Document the result.** Either:
   - Hypothesis confirmed → all 3 deployment options below are valid
   - Hypothesis wrong → revert all the sign flips, dig deeper

### Why this verification matters

If the hypothesis is WRONG (e.g., the visual mismatch comes from
something else, like a different DH parameter or a calibration
artifact specific to certain joint configurations), then our sign
flip would have caused our sim to match real only by COINCIDENCE
at HOME — and any other pose would be off in a different way.
We'd be fixing the symptom, not the cause.

The verification MUST happen at the cell before sending any motion
to the real robot via our scripts.

## ⚠️ FOLLOW-UP 2026-05-26 (late evening) — TRUE root cause found, axis-flip-only fix was insufficient

The "axis-flip-only" fix below worked at HOME by exploiting a symmetry
coincidence, BUT at any non-HOME pose the URDF visualization is mirrored
from the real cabinet — verified empirically with `tcp_compare.py`:

```
                 HOME |Δ|   EXTREME |Δ|
axis-flip only   0.4 mm     854 mm    ← matches at HOME by symmetry only
true fix (B)     0.4 mm     0.4 mm    ← matches everywhere
```

`calibration/fk_experiment.py` runs URDF FK under four hypotheses and
proves the actual fix is:

1. **REMOVE** the 180° yaw on `base_link → base_link_inertia` in
   `ur_macro.xacro:350` (was `rpy="0 0 ${pi}"`, now `rpy="0 0 0"`).
2. **REVERT** the shoulder_pan axis flip in `ur_macro.xacro:364`
   (was `<axis xyz="0 0 -1"/>` from the symptom-patch, now back to
   `<axis xyz="0 0 1"/>` per upstream).

These two changes cancel out at HOME (both fix and bug give the same
HOME tool0) but ONLY the proper fix works at every other pose.

HOME_Q in all scripts/SRDF/initial_positions stays at `[+π/2, -π/2, -π/2,
-π/2, +π/2, +π/2]` — no change needed there. The cabinet's RTDE convention
and the URDF now agree on what shoulder_pan = +π/2 means physically.

The upstream comment at `ur_macro.xacro:345-348` ("'base_link' is REP-103
aligned ... internal frames of the robot/controller have X+ pointing
backwards") is wrong for THIS cabinet's primary-interface output — our
cabinet exposes kinematic data already in REP-103 base frame.

See `calibration/fk_experiment.py` for the empirical proof script and
`calibration/README.md` for the wider calibration-extraction context.

---

## ✅ RESOLVED 2026-05-26 — URDF axis fixed, sim and real now agree

**Final fix applied.** After confirming via `/joint_states` that the
ur_robot_driver passes the cabinet's `+π/2` shoulder_pan value through
unchanged (no sign correction), we chose Option 3 from below: fix the
URDF axis convention so sim and real both use `+π/2` for HOME.

### What changed

1. **`src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro:356`**
   Inverted the `shoulder_pan_joint` axis:
   ```xml
   <!-- before -->
   <axis xyz="0 0 1" />
   <!-- after -->
   <axis xyz="0 0 -1" />
   ```
2. Reverted every script/config back to `+π/2`:
   - `src/ur10e_rg6_moveit_config/config/ur10e_rg6.srdf` home group_state
   - `src/ur10e_rg6_moveit_config/config/initial_positions.yaml`
   - `tests/play_pickplace.py` HOME_Q[0]
   - `tests/real_hw_smoke.py` HOME_Q[0]

### Why this is the right fix

- Pendant, RTDE, `/joint_states`, `dodectest3.urp`, Grasshopper outputs —
  every external interface uses the cabinet's `+π/2` convention.
- The only outlier was our URDF. Flipping the joint axis aligns the
  URDF's local convention with the cabinet's without touching any
  external interface.
- No sign-flip helper is needed in any script. No re-teach of the
  cabinet HOME is needed.
- Sim and real both visualize and command HOME at `+π/2`.

### Risk — `ur_macro.xacro` is vcs-imported

The file lives under `src/Universal_Robots_ROS2_Description/` and is
managed by `ros2.repos`. **A `vcs import src < ros2.repos --force` will
overwrite this edit.** If that happens:
- Re-apply the one-line axis change (see "What changed" above).
- The comment in `ur_macro.xacro:356` flags the edit for greppability.

The (untried) longer-term alternative is to fork that package into our
own repo and bump `ros2.repos` to point at the fork.

### What "Critical caveat for real hardware" used to say

The text below documented the three deployment options when this issue
was open. Kept for traceability; superseded by the resolution above.

---

## Critical caveat for real hardware (HISTORICAL — superseded by resolution above)

**The fix is SIM-only.** On the real cabinet:
- The URScript HOME (verified in `dodectest3.urp`) is `[+pi/2, -pi/2, -pi/2, -pi/2, +pi/2, +pi/2]`
- Sending our new `HOME_Q = [-pi/2, ...]` to the cabinet will rotate the
  arm to the OPPOSITE side from the operator's known HOME
- The work table is on the +pi/2 side (per operator), so `-pi/2` would
  send the arm AWAY from the work area

### Two options for real-hardware deployment:

**Option 1 — Re-teach the cabinet's HOME:** Use the pendant to record a
new HOME at `[-pi/2, -pi/2, -pi/2, -pi/2, +pi/2, +pi/2]`. The operator
adopts the new convention. Risk: other programs / URPs that reference
the old HOME break. Best if this cell's URP library is small.

**Option 2 — Have separate sim and real HOME_Q in scripts:** Add a
`--real-hw` flag (or detect `use_fake_hardware:=false`) and substitute
`+pi/2` for the shoulder_pan value before sending to the real driver.
Keeps both sim visualization and real-hardware compatibility. Risk: the
RViz visualization during real-hardware runs will then be wrong again
(but the actual motion is correct).

**Option 3 — Fix the URDF's joint axis convention properly:** Investigate
why our URDF and the cabinet disagree on shoulder_pan direction. Fix
the underlying URDF so both sim and real use `+pi/2` consistently. Most
correct, most work.

For now: scripts have `-pi/2`. Real-hardware deployment must address
the caveat before Phase 5+ of the validation plan can use real motion.

## Related

- [real_hw_connection.md](real_hw_connection.md) — port + URCap details
- [rviz_visual_orientation_mismatch.md](rviz_visual_orientation_mismatch.md)
  — the longer attempt log (largely superseded by this finding)
- [`tests/measure_real_robot_pose.py`](../tests/measure_real_robot_pose.py)
  — the diagnostic that first quantified the visual mismatch
- `D:\robot_ws\reference\dodectest3.urp` — operator-authored URScript
  HOME values (`+pi/2` shoulder_pan)

## Last updated

2026-05-26.

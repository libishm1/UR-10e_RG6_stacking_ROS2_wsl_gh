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

## Critical caveat for real hardware

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

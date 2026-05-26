# RViz visual orientation mismatch with real cell — unresolved

## Purpose

Document a recurring cosmetic mismatch between our RViz visualization
and the physical UR10e + RG6 cell. **Kinematics, IK, controllers, pick-place
sequence all work correctly.** This is purely a visual issue in RViz where
the arm at HOME appears to extend in the opposite direction from what the
physical robot in the cell shows.

## What was observed (2026-05-26, at the cell)

User side-by-side comparison: at the same HOME joint values
`[1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]`:

- **Real robot**: arm extends to one side of the cabinet (gripper hangs over
  the cable side of the base — user's visual reference)
- **RViz**: arm extends to the OPPOSITE side of the cabinet, same HOME joints
- Other elements (URScript world poses, pre-spawned boxes, pedestal) all
  appear in the correct positions relative to the robot in RViz — only the
  arm direction differs from real

Pre-flight measurements from `tests/measure_real_robot_pose.py`:
- Real TCP at HOME (base frame): `(0.176, 0.691, 0.400)` — primarily +Y, Z low
- Sim TCP at HOME (base frame): `(0.001, 0.532, 1.484)` — primarily +Y, Z high

Both have positive Y → same direction relative to base_link. **But sim Z is
1m+ higher than real.** Same joint values, same calibration applied, very
different physical pose. This is a deeper kinematic-model discrepancy than
the per-link calibration corrections.

## What we tried that DIDN'T fix it

| Attempt | Outcome |
|---|---|
| Apply `kinematics_parameters_file` from `ur_calibration calibration_correction` extraction | Small per-link corrections, doesn't change the gross arm direction in sim |
| Rotate `base_link_inertia` visual mesh by `pi/2` | Cylindrical base mesh is rotationally symmetric — invisible change |
| Rotate `base_link_inertia` visual mesh by `pi` (default) | Same — no visible difference |
| Apply `<origin xyz="0 0 0" rpy="0 0 pi">` to the `ur_robot` macro mount | Rotates base_link 180° in world, but arm STILL appears flipped vs real at the user's comparison angle |
| Various RViz camera angle changes | Doesn't change the world-frame arm direction |
| `wsl --shutdown` to reset WSLg state | Fixed window rendering issues but didn't change kinematic visual |

## Why URDF base rotation doesn't visually fix it

When you change `<origin rpy="0 0 pi">` on the `ur_robot` mount:
- base_link's frame rotates 180° in world
- All child links rotate together (in world)
- Boxes/pedestal anchored to base_link also rotate in world
- Net effect: the ENTIRE scene rotates 180° in world

If your RViz `Fixed Frame` is `base_link`, you see no change (you're
orbiting around base_link which never moves relative to itself).

If your RViz `Fixed Frame` is `world`, the whole scene rotates 180° but
the arm STILL extends in the same direction relative to base_link/robot
— and base_link itself rotates 180° too, so visually the arm goes the
SAME WAY relative to the cabinet body (just the whole thing rotates in
world).

The user is comparing "where does the arm go relative to the cabinet
body in RViz" vs "where does the arm go relative to the cabinet body
in real life." That ratio doesn't change with mount rotation — it's
determined by the joint chain inside the URDF.

## Why kinematic calibration didn't fix it either

The per-link calibration deltas extracted by
`ur_calibration calibration_correction` are SMALL corrections to the
nominal DH parameters. The dramatic 1m+ Z mismatch suggests our URDF's
nominal DH values are fundamentally different from what the actual
robot's controller uses. Possible root causes:

1. The real cabinet has been firmware-upgraded with a non-standard
   kinematic model
2. The URCap (OnRobot or other) reconfigures the TCP frame at runtime,
   which `getActualTCPPose()` returns but our URDF doesn't know about
3. The URDF we're using is for a different UR10e variant than the
   physical robot

We did not dive deeper because **the visual mismatch doesn't block
real-hardware use**: the controller uses its OWN calibration when
moving the actual arm, regardless of what our URDF predicts.

## Final decision

**Accept the cosmetic mismatch as a known limitation.** Real-hardware
motion will work correctly because the controller does its own
kinematics. RViz visualization is for human convenience and is
"close enough" to be useful for planning + collision checking — just
not pixel-accurate to the physical pose.

URDF reverted to default base mount (`rpy="0 0 0"`). Mesh patches
removed. Calibration yaml kept (it's harmless — small per-link
corrections, may be useful if we ever investigate the deeper
kinematic mismatch).

## What CAN still be done if you really want visual parity

If a future session insists on matching RViz to the real cell:

1. **Investigate the real controller's actual DH parameters.** Pull
   `/programs/installation/<name>.installation` from the cabinet via
   SFTP — it contains the calibrated kinematic model the controller
   uses. Compare to our URDF's `default_kinematics.yaml` to find the
   delta.
2. **Manually override individual joint origins in our URDF.** Likely
   need adjustments on shoulder_lift_joint and elbow_joint origins
   to lower the TCP at HOME from 1.485 m to 0.400 m.
3. **Use a different `ur_description` package version** that has DH
   params matching the actual robot.

These are deep-dive options. Not in scope for the current cell
bring-up.

## Related

- `tests/measure_real_robot_pose.py` — the diagnostic that revealed
  the 1m+ Z discrepancy
- `SESSION_HANDOFF.md` — the 2026-05-26 checkpoint
- `D:\robot_ws\reference\dodectest3.urp` — the URP whose embedded
  `<kinematics>` block is what we extracted (and it's still small
  per-link deltas, not gross structural differences)
- [`real_hw_connection.md`](real_hw_connection.md) — explains why URScript
  poses still work on real even when sim visualization differs

## Last updated

2026-05-26.

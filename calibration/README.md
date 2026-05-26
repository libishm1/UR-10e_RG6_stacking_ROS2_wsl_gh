# Cabinet calibration extraction — DEEP CALIBRATION TASK

Raw cabinet kinematic calibration data for offline analysis and a
custom URDF extractor that bypasses the broken Euler-angle round-trip
in upstream `ur_calibration`.

## Files

| File | Source | Purpose |
|---|---|---|
| `cabinet_calibration.conf` | `root@192.168.1.100:/root/.urcontrol/calibration.conf` | Raw DH calibration deltas (a/d/alpha/theta) — this is the ground truth |
| `cabinet_firmware_appinfo.yaml` | `root@192.168.1.100:/root/.urcontrol/firmware_appinfo.yaml` | Cabinet firmware/model metadata (UR10e G5 e-series, s/n 20255201551) |
| (see `src/ur10e_rg6_moveit_config/config/ur10e_cell_calibration.yaml`) | Output of upstream `ur_calibration calibration_correction` | What we currently use in the URDF — has the Euler-decomposition bug |

## Why we need a custom extractor

The upstream `ur_calibration` tool is BUG'd for this cabinet, specifically:

1. `Calibration::correctChain()` in `src/calibration.cpp:59` correctly
   absorbs `delta_theta[1]` (-1.408 rad) and `delta_theta[2]` (+1.372 rad)
   into the DH chain via `correctAxis(1)` and `correctAxis(2)`.
2. BUT `Calibration::toYaml()` at `src/calibration.cpp:230` decomposes the
   resulting rotation matrix via `Eigen::eulerAngles(0, 1, 2)`, which
   returns NON-UNIQUE Euler triples for rotations near π or near gimbal
   lock. For our cabinet the `forearm` link comes out as `(π, π, π)` —
   mathematically a valid representation of *some* 180° rotation, but
   URDF parses RPY sequentially and reconstructs a different rotation
   matrix from those three angles.
3. Net effect: URDF link `forearm` is rotated relative to the cabinet's
   actual `forearm` orientation. Combined with `correctAxis(2)`'s
   downstream changes, this compounds into a **705 mm position error**
   at non-HOME poses, with `tcp_compare.py` confirming X/Y agree at
   HOME (symmetric, errors cancel) and diverge wildly at off-HOME.

## Empirical evidence

`tcp_compare.py` results — URDF FK vs cabinet RTDE TCP:

| Pose | shoulder_pan | shoulder_lift | wrist_2 | ΔX (m) | ΔY (m) | ΔZ (m) |
|---|---|---|---|---|---|---|
| HOME | +1.5708 | -1.5708 | +1.5708 | +0.001 | 0.000 | +0.279 (set_tcp) |
| Mild off-HOME | +1.5822 | -1.8390 | +1.5708 | +0.018 | -0.004 | +0.279 (set_tcp) |
| Extreme | +2.4025 | -1.6263 | -0.8636 | **+0.575** | **-0.388** | -0.130 (no longer +0.279!) |

The Z delta breaks out of the constant `set_tcp` offset at the extreme
pose — that's not a TCP offset issue, it's an actual kinematic error.

## Raw `cabinet_calibration.conf` parsed values

```
delta_theta = [
   -1.14e-07,        # j0 shoulder_pan  — ~0, no effect
   -1.408,           # j1 shoulder_lift — HUGE (-81°)
   +1.372,           # j2 elbow         — HUGE (+79°)
   +0.036,           # j3 wrist_1       — small but non-zero
   -9.45e-08,        # j4 wrist_2       — ~0
   +9.36e-08,        # j5 wrist_3       — ~0
]
delta_a = [+4.1e-05, +0.513, +0.000452, +3.0e-05, +2.3e-05, 0]
delta_d = [+9.5e-05, +429.4, -433.1, +3.68, -8.96e-05, -0.000988]   # mm
delta_alpha = [-0.000615, -0.00141, +0.00553, -0.000393, +0.000662, 0]
```

`delta_d[1]` and `delta_d[2]` (the +429.4 mm / -433.1 mm) are absorbed
by `correctAxis` via `d → 0` redistribution into `a` and `theta`. That
part is correct — the bug is purely in the rotation extraction.

## Next-session task: `apply_calibration_quaternion.py`

A custom extractor that reproduces `correctChain()` then outputs each
link's `<origin>` using a **quaternion** intermediate (or equivalent
matrix-direct URDF emission), bypassing Euler decomposition.

Output format: directly modify `ur10e_rg6.urdf.xacro` (or generate a
new yaml that the upstream `xacro:read_model_data` consumer can ingest)
so that each link's frame matches the cabinet's actual `correctAxis`-output
matrix bit-for-bit.

Pseudocode:
```python
# 1. Parse cabinet_calibration.conf into delta_a, delta_d, delta_alpha, delta_theta
# 2. Build nominal UR10e DH segments (from physical_parameters.yaml)
# 3. Apply deltas: segments[i].(a,d,alpha,theta) += deltas[i]
# 4. Run correctAxis(1) and correctAxis(2) — port from C++ calibration.cpp
# 5. For each link, get the simplified chain matrix (chain_[2i+1] * chain_[2i+2])
# 6. Extract quaternion (NOT Euler), write directly into URDF as
#    <origin xyz="x y z" rpy="..."> where rpy is computed via
#    quaternion → axis-angle → fixed-axis Tait-Bryan, with sign
#    disambiguation that respects the original matrix
# 7. Verify by reconstructing URDF FK and comparing to cabinet's
#    forward kinematics at multiple joint configurations
```

Validation: at any joint configuration `q`, our URDF FK at q must equal
the cabinet's RTDE TCP at q (modulo the `set_tcp` offset). Use
`tcp_compare.py` at >= 4 distinct poses to confirm.

## Workarounds until then

1. **ROS workspace as sim + Path B for real motion** — the Grasshopper
   + Robots.NET + Path B pipeline doesn't touch our URDF; the cabinet
   uses its own (correct) kinematics. This is the safe production path.
2. **Joint-space-only ROS motion** — `direct_trajectory_smoke.py`
   style. Each waypoint is a joint vector pre-computed in the cabinet's
   joint space; no URDF FK/IK in the loop. Loses MoveIt's Cartesian
   planning but motion executes correctly.
3. **Do NOT use MoveIt Cartesian/LIN on real hardware** until calibration
   is fixed — the IK gives plausible joint values but the cabinet ends
   up in a different pose than the URDF predicted.

## Last updated

2026-05-26 (late evening).

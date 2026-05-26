#!/usr/bin/env python3
"""Custom calibration extractor — bypasses ur_calibration's Euler-decomposition bug.

Reads cabinet_calibration.conf, applies DH deltas, runs the equivalent of
Calibration::correctChain() (axes 1 and 2), then emits each URDF link's
transform using a ROBUST matrix-to-rpy extraction (ZYX Euler with pitch
constrained to [-π/2, π/2]), avoiding the (π, π, π) ambiguity that
Eigen::eulerAngles(0, 1, 2) produces for our cabinet's 180°-class
rotation matrices.

Output: ur10e_cell_calibration_fixed.yaml — drop-in replacement for the
upstream ur_calibration extractor's output.

See calibration/README.md for the full motivation.
"""
import math
import re
import sys
from pathlib import Path
from typing import List

import numpy as np
import yaml

HERE = Path(__file__).parent
CALIBRATION_CONF = HERE / "cabinet_calibration.conf"
OUTPUT_YAML = HERE / "ur10e_cell_calibration_fixed.yaml"

# UR10e nominal DH (a, d, alpha, theta) — joint indices 0..5
# Matches values in src/Universal_Robots_ROS2_Driver/ur_calibration's
# default DHRobot constructor for UR10e. SI units.
NOMINAL_DH_UR10E = [
    # (a,        d,        alpha,        theta)
    (0.0,      0.1807,   0.0,           0.0),   # j0 shoulder_pan
    (0.0,      0.0,     +math.pi / 2,   0.0),   # j1 shoulder_lift
    (-0.6127,  0.0,      0.0,           0.0),   # j2 elbow
    (-0.57155, 0.17415,  0.0,           0.0),   # j3 wrist_1
    (0.0,      0.11985, +math.pi / 2,   0.0),   # j4 wrist_2
    (0.0,      0.11655, -math.pi / 2,   0.0),   # j5 wrist_3
]

# URDF link names (matches ur_calibration::link_names_ in calibration.hpp)
LINK_NAMES = ["shoulder", "upper_arm", "forearm",
              "wrist_1", "wrist_2", "wrist_3"]


# ---------------------------------------------------------------------
# Calibration file parsing
# ---------------------------------------------------------------------

def _parse_array(line: str) -> List[float]:
    m = re.search(r"\[([^\]]+)\]", line)
    if not m:
        raise ValueError(f"no array in line: {line!r}")
    return [float(x.strip()) for x in m.group(1).split(",")]


def parse_calibration(path: Path) -> dict:
    """Read /root/.urcontrol/calibration.conf-format file. Returns a dict
    with the four 6-element arrays delta_theta, delta_a, delta_d, delta_alpha."""
    result = {}
    section = None
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("[") and s.endswith("]"):
            section = s[1:-1]
            result["_section"] = section
            continue
        if "=" in s:
            key, _, value = s.partition("=")
            key = key.strip()
            if key in ("delta_theta", "delta_a", "delta_d", "delta_alpha"):
                result[key] = _parse_array(value)
    for k in ("delta_theta", "delta_a", "delta_d", "delta_alpha"):
        if k not in result or len(result[k]) != 6:
            raise ValueError(f"malformed {k} in {path}")
    return result


# ---------------------------------------------------------------------
# 4x4 matrix builders (numpy)
# ---------------------------------------------------------------------

def _rotz_3(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)


def _rotx_3(alpha: float) -> np.ndarray:
    c, s = math.cos(alpha), math.sin(alpha)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]], dtype=float)


def _seg_d_theta(theta: float, d: float) -> np.ndarray:
    """seg1: rotation Rz(theta), translation (0, 0, d). Matches
    ur_calibration::buildChain() seg1_mat."""
    M = np.eye(4)
    M[:3, :3] = _rotz_3(theta)
    M[2, 3] = d
    return M


def _seg_a_alpha(alpha: float, a: float) -> np.ndarray:
    """seg2: rotation Rx(alpha), translation (a, 0, 0). Matches
    ur_calibration::buildChain() seg2_mat."""
    M = np.eye(4)
    M[:3, :3] = _rotx_3(alpha)
    M[0, 3] = a
    return M


# ---------------------------------------------------------------------
# Chain build + correctChain port
# ---------------------------------------------------------------------

def build_chain(dh_params: List[tuple]) -> List[np.ndarray]:
    """Build the 12-entry chain matching ur_calibration::buildChain().
    chain[2i] = seg_d_theta for joint i; chain[2i+1] = seg_a_alpha for joint i."""
    chain = []
    for i in range(6):
        a, d, alpha, theta = dh_params[i]
        chain.append(_seg_d_theta(theta, d))
        chain.append(_seg_a_alpha(alpha, a))
    return chain


def correct_axis(chain: List[np.ndarray], link_index: int,
                 original_theta: float, original_alpha: float) -> None:
    """In-place port of ur_calibration::correctAxis(). Absorbs the large
    d offset at chain[2*link_index] into the surrounding a/theta of the
    same segment + the d of the next segment, leaving kinematics
    unchanged but URDF-friendly (each link has a single offset transform)."""
    d_theta_segment = chain[2 * link_index]
    a_alpha_segment = chain[2 * link_index + 1]

    d = d_theta_segment[2, 3]
    a = a_alpha_segment[0, 3]

    if abs(d) < 1e-12:
        return  # nothing to do

    next_joint_root = d_theta_segment @ a_alpha_segment
    next_root_position = next_joint_root[:3, 3].copy()

    next_d_theta_segment = chain[(link_index + 1) * 2]
    next_d_theta_end = (next_joint_root @ next_d_theta_segment)[:3, 3]

    direction = next_d_theta_end - next_root_position

    # Intersect the parametrised line (P = next_root_position + t * direction)
    # with the XY plane z = 0.
    if abs(direction[2]) < 1e-12:
        # Parallel to XY plane — shouldn't happen for UR DH structure.
        raise RuntimeError(f"correct_axis({link_index}): rotation axis parallel to XY plane")

    intersection_param = -next_root_position[2] / direction[2]
    intersection_point = next_root_position + intersection_param * direction

    subtraction_angle = math.pi if abs(a) > 0 else 0.0
    new_theta = math.atan2(intersection_point[1], intersection_point[0]) - subtraction_angle
    new_link_length = -1.0 * float(np.linalg.norm(intersection_point))

    sign_dir = 1.0 if direction[2] > 0 else -1.0
    distance_correction = intersection_param * sign_dir

    # In-place modify d_theta_segment: d → 0, rotation → Rz(new_theta)
    d_theta_segment[:3, :3] = _rotz_3(new_theta)
    d_theta_segment[2, 3] = 0.0

    # In-place modify a_alpha_segment: a → new_link_length,
    # rotation → Rz(theta_orig − new_theta) * Rx(alpha)
    a_alpha_segment[:3, :3] = _rotz_3(original_theta - new_theta) @ _rotx_3(original_alpha)
    a_alpha_segment[0, 3] = new_link_length

    # Compensate next segment's d
    chain[2 * link_index + 2][2, 3] -= distance_correction


def simplify_chain(chain: List[np.ndarray]) -> List[np.ndarray]:
    """Port of ur_calibration::getSimplified(). Produces 7 matrices:
    [chain[0],  chain[1]*chain[2],  chain[3]*chain[4], ...,  chain[11]].
    The first 6 are the per-link URDF transforms (shoulder, upper_arm,
    forearm, wrist_1, wrist_2, wrist_3). The 7th is the wrist_3 → flange
    a_alpha leftover and is NOT used in the URDF yaml."""
    simplified = [chain[0]]
    for i in range(1, 11, 2):
        simplified.append(chain[i] @ chain[i + 1])
    simplified.append(chain[-1])
    return simplified


# ---------------------------------------------------------------------
# Robust matrix → URDF rpy extraction (the part the upstream tool got wrong)
# ---------------------------------------------------------------------

def matrix_to_urdf_rpy(R: np.ndarray) -> tuple:
    """Extract (roll, pitch, yaw) such that R == Rz(yaw) · Ry(pitch) · Rx(roll).
    This is the URDF / REP-103 convention. Pitch is constrained to (-π/2, π/2);
    at exactly ±π/2 (gimbal lock) the (roll, yaw) split is conventional.

    Unlike Eigen::eulerAngles(0, 1, 2) which can return (π, π, π) for an
    arbitrary 180° rotation, this function returns a small-angle
    representation when one exists, and matches URDF parser's reconstruction
    exactly."""
    # Standard ZYX Euler from rotation matrix.
    # R[2,0] = -sin(pitch)
    # If cos(pitch) != 0:  roll = atan2(R[2,1], R[2,2]);  yaw = atan2(R[1,0], R[0,0])
    sy_sq = R[0, 0] ** 2 + R[1, 0] ** 2
    cos_pitch = math.sqrt(sy_sq)
    pitch = math.atan2(-R[2, 0], cos_pitch)
    if cos_pitch > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: pitch == ±π/2. Conventional choice: yaw = 0, fold into roll.
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def verify_rpy_roundtrip(R_orig: np.ndarray, roll: float, pitch: float, yaw: float,
                          tol: float = 1e-6) -> tuple:
    """Verify that the URDF reconstruction Rz(yaw)·Ry(pitch)·Rx(roll) == R_orig.
    Returns (max_diff, R_reconstructed). Use to catch any extraction error."""
    R_recon = (_rotz_3(yaw) @
               np.array([[math.cos(pitch), 0, math.sin(pitch)],
                         [0, 1, 0],
                         [-math.sin(pitch), 0, math.cos(pitch)]]) @
               _rotx_3(roll))
    return float(np.max(np.abs(R_recon - R_orig))), R_recon


# ---------------------------------------------------------------------
# Main extractor
# ---------------------------------------------------------------------

def main():
    if not CALIBRATION_CONF.exists():
        print(f"Missing: {CALIBRATION_CONF}", file=sys.stderr)
        sys.exit(1)

    raw = parse_calibration(CALIBRATION_CONF)

    # Apply deltas to nominal DH
    dh = []
    for i in range(6):
        a_n, d_n, alpha_n, theta_n = NOMINAL_DH_UR10E[i]
        dh.append((a_n + raw["delta_a"][i],
                   d_n + raw["delta_d"][i],
                   alpha_n + raw["delta_alpha"][i],
                   theta_n + raw["delta_theta"][i]))

    # Snapshot original theta and alpha BEFORE correctAxis touches them
    original_thetas = [dh[i][3] for i in range(6)]
    original_alphas = [dh[i][2] for i in range(6)]

    print("=== Calibrated DH (nominal + deltas) ===")
    for i in range(6):
        a, d, alpha, theta = dh[i]
        print(f"  j{i}  a={a:+.6f}  d={d:+.6f}  α={alpha:+.6f}  θ={theta:+.6f}")

    # Build chain and run correctChain
    chain = build_chain(dh)

    # correctChain only touches axes 1 and 2 (shoulder_lift, elbow) per the C++
    correct_axis(chain, 1, original_thetas[1], original_alphas[1])
    correct_axis(chain, 2, original_thetas[2], original_alphas[2])

    simplified = simplify_chain(chain)

    # Emit URDF link transforms using robust rpy extraction
    output = {"kinematics": {}}
    print()
    print("=== Per-link URDF transforms (post-correctChain, robust rpy) ===")
    max_roundtrip = 0.0
    for i, name in enumerate(LINK_NAMES):
        M = simplified[i]
        x, y, z = float(M[0, 3]), float(M[1, 3]), float(M[2, 3])
        R = M[:3, :3]
        roll, pitch, yaw = matrix_to_urdf_rpy(R)
        # Verify the round-trip matches (this is what upstream got wrong)
        diff, _ = verify_rpy_roundtrip(R, roll, pitch, yaw)
        max_roundtrip = max(max_roundtrip, diff)

        output["kinematics"][name] = {
            "x": x, "y": y, "z": z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
        }
        print(f"  {name:10s}  xyz=({x:+.6f},{y:+.6f},{z:+.6f})  "
              f"rpy=({roll:+.6f},{pitch:+.6f},{yaw:+.6f})  "
              f"roundtrip_err={diff:.2e}")

    output["kinematics"]["hash"] = "calib_custom_quaternion_extractor_v1"

    # Sanity check: round-trip should be machine epsilon
    if max_roundtrip > 1e-9:
        print(f"\nWARNING: max rpy round-trip error {max_roundtrip:.2e} "
              "(should be ~1e-15). Something off in the extractor.")
    else:
        print(f"\nRoundtrip OK (max err {max_roundtrip:.2e} — machine epsilon).")

    # Write yaml
    with OUTPUT_YAML.open("w") as f:
        yaml.dump(output, f, default_flow_style=False, sort_keys=False)
    print(f"\nWrote: {OUTPUT_YAML}")


if __name__ == "__main__":
    main()

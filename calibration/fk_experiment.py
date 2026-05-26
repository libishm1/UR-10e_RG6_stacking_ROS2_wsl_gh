#!/usr/bin/env python3
"""Python forward kinematics experiment for the UR10e using the existing
ur10e_cell_calibration.yaml link transforms. Tests two hypotheses for the
URDF↔cabinet mismatch:

  HYPOTHESIS A — current URDF (with 180° base→base_link_inertia yaw AND
                 shoulder_pan axis flipped to -Z)
  HYPOTHESIS B — no 180° yaw at base, shoulder_pan axis +Z (Tesseract-style)
  HYPOTHESIS C — no 180° yaw, shoulder_pan axis -Z
  HYPOTHESIS D — 180° yaw applied, shoulder_pan axis +Z (pre my-fix state)

For each hypothesis, computes URDF FK at two joint configurations we measured
on the real cabinet and prints predicted TCP vs measured TCP. Whichever
hypothesis gives near-zero deltas at BOTH poses is the right URDF structure.

Read-only — does not touch the running stack.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).parent
YAML_PATH = HERE.parent / "src" / "ur10e_rg6_moveit_config" / "config" / "ur10e_cell_calibration.yaml"

# Measured joint state at HOME (read from /joint_states with /tcp_pose_broadcaster):
#   HOME q (rad): [+1.5708, -1.5708, -1.5708, -1.5708, +1.5708, +1.5708]
#   Measured TCP (m): (+0.1760, +0.6916, +0.4000)   <- cabinet RTDE
#
# Extreme freedrive pose:
#   q (rad): [+2.4025, -1.6263, -1.0774, -1.0961, -0.8636, +1.5825]
#   Measured TCP (m): (+0.0029, +0.5777, +1.3147)   <- cabinet RTDE
#
# Cabinet's set_tcp() applies a +0.279 m offset along the gripper Z axis,
# so the bare-flange tool0 measured Z would be HIGHER than the TCP value;
# we'll compute tool0 FK and then add the set_tcp offset for comparison.
TEST_POSES = [
    {
        "name": "HOME",
        "q": [+1.5708, -1.5708, -1.5708, -1.5708, +1.5708, +1.5708],
        "real_tcp_xyz": (+0.1760, +0.6916, +0.4000),  # set_tcp applied
    },
    {
        "name": "EXTREME",
        "q": [+2.4025, -1.6263, -1.0774, -1.0961, -0.8636, +1.5825],
        "real_tcp_xyz": (+0.0029, +0.5777, +1.3147),
    },
]

LINK_NAMES = ["shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"]
# Joint axis sign (Z component): per ur_macro.xacro
JOINT_AXIS_Z_CURRENT = [-1, 1, 1, 1, 1, 1]      # post my shoulder_pan flip
JOINT_AXIS_Z_ORIG    = [+1, 1, 1, 1, 1, 1]      # upstream default


def rotz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=float)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy: R = Rz(yaw) · Ry(pitch) · Rx(roll). Returns 4x4 homogeneous."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    M = np.eye(4)
    M[:3, :3] = R
    return M


def transl(x: float, y: float, z: float) -> np.ndarray:
    M = np.eye(4)
    M[:3, 3] = [x, y, z]
    return M


def load_link_transforms(yaml_path: Path) -> dict:
    """Load per-link xyz/rpy from the calibration yaml."""
    data = yaml.safe_load(yaml_path.read_text())
    kin = data["kinematics"]
    out = {}
    for name in LINK_NAMES:
        e = kin[name]
        # URDF joint origin is: translate (xyz) then rotate (rpy)
        M = transl(e["x"], e["y"], e["z"]) @ rpy_to_matrix(e["roll"], e["pitch"], e["yaw"])
        out[name] = M
    return out


def forward_kinematics(q: list, links: dict, base_yaw_180: bool, axis_z_signs: list) -> np.ndarray:
    """Compute base_link → tool0 transform. q is the 6-element joint vector.

    Chain (matches our ur_macro.xacro):
      base_link
       → (base_inertia_joint: rpy 0 0 π if base_yaw_180)
      base_link_inertia
       → shoulder_pan_joint at links['shoulder'] origin, axis [0,0,axis_z_signs[0]]
      shoulder_link
       → shoulder_lift_joint at links['upper_arm'] origin, axis [0,0,axis_z_signs[1]]
      upper_arm_link
       → elbow_joint at links['forearm'], axis [0,0,axis_z_signs[2]]
      forearm_link
       → wrist_1_joint at links['wrist_1'], axis [0,0,axis_z_signs[3]]
      wrist_1_link
       → wrist_2_joint at links['wrist_2'], axis [0,0,axis_z_signs[4]]
      wrist_2_link
       → wrist_3_joint at links['wrist_3'], axis [0,0,axis_z_signs[5]]
      wrist_3_link → (identity) → flange → (identity) → tool0
    """
    T = np.eye(4)
    if base_yaw_180:
        T = T @ rpy_to_matrix(0, 0, math.pi)
    for i, name in enumerate(LINK_NAMES):
        T = T @ links[name]                    # joint origin
        T = T @ rotz(q[i] * axis_z_signs[i])   # joint rotation about its axis
    # flange and tool0 are identity from wrist_3 in our URDF
    return T


HYPOTHESES = [
    ("A (current URDF: 180° yaw + shoulder_pan axis -Z)", True,  JOINT_AXIS_Z_CURRENT),
    ("B (Tesseract-style: no yaw + shoulder_pan axis +Z)", False, JOINT_AXIS_Z_ORIG),
    ("C (no yaw + shoulder_pan axis -Z)",                  False, JOINT_AXIS_Z_CURRENT),
    ("D (yaw + shoulder_pan axis +Z, pre-fix state)",       True,  JOINT_AXIS_Z_ORIG),
]


def main():
    if not YAML_PATH.exists():
        print(f"Missing: {YAML_PATH}", file=sys.stderr)
        sys.exit(1)
    links = load_link_transforms(YAML_PATH)

    print(f"Loaded link transforms from: {YAML_PATH.name}")
    print()
    for pose in TEST_POSES:
        q = pose["q"]
        real_xyz = pose["real_tcp_xyz"]
        # Cabinet's set_tcp adds ~+0.279 m offset along the gripper Z axis,
        # but its DIRECTION in world frame depends on the wrist orientation.
        # For comparison we compute tool0 position (URDF FK gives flange/tool0
        # directly — no set_tcp). The cabinet's real_tcp is the gripper TCP.
        # So real_tcp = tool0 + R_tool0 @ (0, 0, 0.279).
        # For HOME, R_tool0's Z aligns with world -Z, so real_z = tool0_z - 0.279.
        # We'll compute and report tool0_xyz; user can mentally add/subtract 0.279.
        print(f"=== pose: {pose['name']}  q = {[f'{x:+.4f}' for x in q]} ===")
        print(f"  real cabinet TCP (set_tcp applied): {real_xyz}")
        for label, yaw180, signs in HYPOTHESES:
            T = forward_kinematics(q, links, yaw180, signs)
            tool0 = T[:3, 3]
            # Translate tool0 by the gripper Z axis (which is the 3rd column of T[:3,:3])
            grip_z_world = T[:3, 2]
            grip_tcp_predicted = tool0 + grip_z_world * 0.279
            dx = grip_tcp_predicted[0] - real_xyz[0]
            dy = grip_tcp_predicted[1] - real_xyz[1]
            dz = grip_tcp_predicted[2] - real_xyz[2]
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            print(f"  HYPOTHESIS {label}")
            print(f"     tool0 = ({tool0[0]:+.4f}, {tool0[1]:+.4f}, {tool0[2]:+.4f})")
            print(f"     +set_tcp = ({grip_tcp_predicted[0]:+.4f}, "
                  f"{grip_tcp_predicted[1]:+.4f}, {grip_tcp_predicted[2]:+.4f})")
            print(f"     Δ = ({dx:+.4f}, {dy:+.4f}, {dz:+.4f})  |Δ|={d:.4f}")
        print()


if __name__ == "__main__":
    main()

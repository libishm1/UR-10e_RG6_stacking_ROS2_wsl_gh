"""Replay the URScript pick-and-place program.

Two gripper modes:
  --sim          (default) publish JointTrajectory to /rg6_gripper_controller
                 /joint_trajectory — drives the ros2_control mock interface
                 in RViz, using the calibrated width→rad cubic.
  --real-gripper drive the OnRobot RG6 over UR Tool I/O via
                 /io_and_status_controller/set_io. Pin 16 (tool digital
                 output 0) HIGH = close, LOW = open. BINARY only — width
                 and force args are ignored (the RG6's onboard MCU picks
                 the default stroke/force when the line goes HIGH). For
                 continuous width control you need either an OnRobot
                 Compute Box (Modbus TCP) or the OnRobot RS-485 URCap.
                 The OnRobot URCap on the pendant must be set to
                 Installation → Tool I/O → Controlled by: User so it
                 doesn't fight these writes.

Maps:
  movel(WP_n)        -> Pilz LIN on ur_manipulator, Cartesian goal (TCP-aware)
  movej(HOME_q)      -> Pilz PTP on ur_manipulator, joint goal
  rg_grip(width_mm)  -> sim:  JointTrajectory using calibrated width→rad
                        real: binary set_io on pin 16 (HIGH=close, LOW=open).
                              width<60mm → close, else → open.
  rg_payload_set()   -> skipped (no ROS 2 equivalent yet)
  sleep(n)           -> time.sleep(n)

URScript poses are [x,y,z,rx,ry,rz] in metres + axis-angle. The TCP offset
(241 mm from tool0 to rg6_tcp) is handled by tcp_to_tool0() — confirmed in
sim against 20 pick-and-place cycles.

Usage (full stack must be running: ur_control + gripper JTC + move_group):
    python3 play_pickplace.py                       # all 20 cycles, sim gripper
    python3 play_pickplace.py --max 2               # first 2 cycles, sim
    python3 play_pickplace.py --real-gripper        # use URCap rg_grip()
    python3 play_pickplace.py --real-gripper --max 3
"""
import argparse
import math
import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Ensure the helper next to this file is importable when run as
# `python3 tests/play_pickplace.py` from the workspace root.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)
from onrobot_io_grip import OnRobotToolIOGrip
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             PositionConstraint, OrientationConstraint,
                             MoveItErrorCodes, CollisionObject,
                             AttachedCollisionObject, PlanningScene)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ---------------- Constants ----------------

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

ARM_GROUP = "ur_manipulator"
TIP_LINK  = "tool0"
BASE_LINK = "base_link"

# URScript poses target the gripper TCP (rg6_tcp), but ur_manipulator's chain
# ends at tool0. The fixed offset rg6_tcp ← tool0 is +0.241 m in tool0's Z
# (bracket 0.051 + grasp_frame 0.190 = 0.241). When we want the TCP at a goal,
# tool0 needs to be 0.241 m closer to base ALONG THE TOOL Z AXIS.
TCP_OFFSET_M = 0.241

GRIPPER_TOPIC = "/rg6_gripper_controller/joint_trajectory"
GRIPPER_JOINT = "rg6_joint"

# URCap (real-hardware) command path
URSCRIPT_TOPIC = "/urscript_interface/script_command"
RG6_DEFAULT_FORCE_N = 40.0   # matches the URScript program's rg_grip(w, 40.0)

# Pick-and-place box visualization
# Boxes are 3 × 5 × 15 cm laid FLAT on the table: the 5×15 face touches z=0,
# the 3-cm axis is vertical. Gripper closes along the 5-cm side from above.
#   Box LOCAL X = 5 cm  (gripper closes around this dimension)
#   Box LOCAL Y = 15 cm (along the finger length)
#   Box LOCAL Z = 3 cm  (vertical / box height)
# Box centroid in world at PICK: (wp_x, wp_y, BOX_Z_M / 2)  — flat on table.
BOX_SIZE_M = (0.05, 0.15, 0.03)
BOX_HEIGHT_M = BOX_SIZE_M[2]
PLANNING_SCENE_TOPIC = "/planning_scene"

# Width-mm → rad mapping. Calibrated empirically by sweeping rg6_joint
# 0..1.3 rad and measuring the world distance between the two flex_finger
# pads (see tests/calibrate_rg6_width.py). The relationship is NEARLY linear
# but INVERTED: angle 0 ≈ 153 mm (full open), angle 1.25 ≈ 1 mm (closed).
# A cubic fit gives <0.5 mm error across the useful range.
#
# Cubic polynomial in width_mm → angle_rad (coefficients from the calibrator).
import yaml as _yaml
import os as _os
_CALIB_PATH = _os.path.join(
    _os.path.dirname(_os.path.abspath(__file__)),
    "../src/ur10e_rg6_moveit_config/config/rg6_width_calibration.yaml")
try:
    with open(_CALIB_PATH) as _f:
        _calib = _yaml.safe_load(_f)["rg6_joint_to_width_mm"]
    _COEFF_ANGLE_OF_WIDTH = _calib["cubic_angle_of_width"]
    _MIN_W = _calib["min_width_mm"]
    _MAX_W = _calib["max_width_mm"]
except Exception:
    # Fallback: derived from the calibration data
    _COEFF_ANGLE_OF_WIDTH = [-2.6099e-07, 4.1813e-05, -0.008620, 1.29513]
    _MIN_W, _MAX_W = 1.16, 153.17


def width_mm_to_angle_rad(width_mm: float) -> float:
    """Inverse mapping from desired finger gap (mm) to rg6_joint command (rad)."""
    w = max(_MIN_W, min(_MAX_W, float(width_mm)))
    a3, a2, a1, a0 = _COEFF_ANGLE_OF_WIDTH
    rad = ((a3 * w + a2) * w + a1) * w + a0
    return max(0.0, min(1.30, rad))

# Safe defaults (per the safety-speed-defaults memory)
# Lowered from 0.15 → 0.08 because the URScript HOME requires a large
# joint-space sweep from boot and the trajectory controller's 0.2 rad
# state tolerance was being violated at higher speeds.
VEL_SCALE = 0.08
ACC_SCALE = 0.08
GRIP_TIME = 1.5
PLAN_TIMEOUT = 20.0   # was 8 — move_group's action server can be slow to accept under load
EXEC_TIMEOUT = 90.0


# ---------------- Waypoints from URScript ----------------
# Each tuple is (x, y, z, rx, ry, rz) — metres + axis-angle radians.
WAYPOINTS = {
    1:  (0.823400,  0.473500,  0.115540, -0.000000,  3.141590,  0.000000),
    2:  (0.823400,  0.473500,  0.028540, -0.000000,  3.141590,  0.000000),
    3:  (0.823400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    4:  (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    5:  (0.658250, -0.226470,  0.436490,  2.796900, -1.430710,  0.000000),
    6:  (0.658250, -0.226470,  0.036490,  2.796900, -1.430710,  0.000000),
    7:  (0.658250, -0.226470,  0.123490,  2.796900, -1.430710,  0.000000),
    8:  (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    9:  (0.723400,  0.473500,  0.115540, -0.000000,  3.141590,  0.000000),
    10: (0.723400,  0.473500,  0.028540, -0.000000,  3.141590,  0.000000),
    11: (0.723400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    12: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    13: (0.657130, -0.331990,  0.436490,  2.796900, -1.430710,  0.000000),
    14: (0.657130, -0.331990,  0.036490,  2.796900, -1.430710,  0.000000),
    15: (0.657130, -0.331990,  0.123490,  2.796900, -1.430710,  0.000000),
    16: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    17: (0.623400,  0.473500,  0.115540, -0.000000,  3.141590, -0.000000),
    18: (0.623400,  0.473500,  0.028540, -0.000000,  3.141590, -0.000000),
    19: (0.623400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    20: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    21: (0.639520, -0.465740,  0.436490,  2.796900, -1.430710, -0.000000),
    22: (0.639520, -0.465740,  0.036490,  2.796900, -1.430710, -0.000000),
    23: (0.639520, -0.465740,  0.123490,  2.796900, -1.430710, -0.000000),
    24: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    25: (0.523400,  0.473500,  0.115540, -0.000000,  3.141590,  0.000000),
    26: (0.523400,  0.473500,  0.028540, -0.000000,  3.141590,  0.000000),
    27: (0.523400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    28: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    29: (0.662730, -0.572290,  0.436490,  2.796900, -1.430710, -0.000000),
    30: (0.662730, -0.572290,  0.036490,  2.796900, -1.430710, -0.000000),
    31: (0.662730, -0.572290,  0.123490,  2.796900, -1.430710, -0.000000),
    32: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    33: (0.423400,  0.473500,  0.115540,  0.000000,  3.141590,  0.000000),
    34: (0.423400,  0.473500,  0.028540,  0.000000,  3.141590,  0.000000),
    35: (0.423400,  0.473500,  0.400000,  0.000000,  3.141590,  0.000000),
    36: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    37: (0.627540, -0.535530,  0.468490,  2.989370,  0.966050, -0.000000),
    38: (0.627540, -0.535530,  0.068490,  2.989370,  0.966050, -0.000000),
    39: (0.627540, -0.535530,  0.155490,  2.989370,  0.966050, -0.000000),
    40: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    41: (0.823400,  0.473500,  0.085540,  0.000000,  3.141590,  0.000000),
    42: (0.823400,  0.473500, -0.001460,  0.000000,  3.141590,  0.000000),
    43: (0.823400,  0.473500,  0.400000,  0.000000,  3.141590,  0.000000),
    44: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    45: (0.648700, -0.398470,  0.468490,  2.989370,  0.966050, -0.000000),
    46: (0.648700, -0.398470,  0.068490,  2.989370,  0.966050, -0.000000),
    47: (0.648700, -0.398470,  0.155490,  2.989370,  0.966050, -0.000000),
    48: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    49: (0.723400,  0.473500,  0.085540,  0.000000,  3.141590,  0.000000),
    50: (0.723400,  0.473500, -0.001460,  0.000000,  3.141590,  0.000000),
    51: (0.723400,  0.473500,  0.400000,  0.000000,  3.141590,  0.000000),
    52: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    53: (0.668050, -0.269300,  0.468490,  2.989370,  0.966050, -0.000000),
    54: (0.668050, -0.269300,  0.068490,  2.989370,  0.966050, -0.000000),
    55: (0.668050, -0.269300,  0.155490,  2.989370,  0.966050, -0.000000),
    56: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440,  0.000000),
    57: (0.623400,  0.473500,  0.085540, -0.000000,  3.141590,  0.000000),
    58: (0.623400,  0.473500, -0.001460, -0.000000,  3.141590,  0.000000),
    59: (0.623400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    60: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    61: (0.657130, -0.331990,  0.500290,  2.796900, -1.430710,  0.000000),
    62: (0.657130, -0.331990,  0.100290,  2.796900, -1.430710,  0.000000),
    63: (0.657130, -0.331990,  0.187290,  2.796900, -1.430710,  0.000000),
    64: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    65: (0.523400,  0.473500,  0.085540, -0.000000,  3.141590,  0.000000),
    66: (0.523400,  0.473500, -0.001460, -0.000000,  3.141590,  0.000000),
    67: (0.523400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    68: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    69: (0.637030, -0.467100,  0.500290,  2.796900, -1.430710, -0.000000),
    70: (0.637030, -0.467100,  0.100290,  2.796900, -1.430710, -0.000000),
    71: (0.637030, -0.467100,  0.187290,  2.796900, -1.430710, -0.000000),
    72: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    73: (0.423400,  0.473500,  0.085540, -0.000000,  3.141590,  0.000000),
    74: (0.423400,  0.473500, -0.001460, -0.000000,  3.141590,  0.000000),
    75: (0.423400,  0.473500,  0.400000, -0.000000,  3.141590,  0.000000),
    76: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
    77: (0.648700, -0.398470,  0.532490,  2.989370,  0.966050, -0.000000),
    78: (0.648700, -0.398470,  0.132490,  2.989370,  0.966050, -0.000000),
    79: (0.648700, -0.398470,  0.219490,  2.989370,  0.966050, -0.000000),
    80: (0.594960, -0.020540,  0.499130,  2.221440,  2.221440, -0.000000),
}
# DRY RUN CLEARANCE — bumps every WAYPOINT Z by this offset for in-air rehearsal.
# Set to 0.0 for normal pick+place contact with the table. Set to ~0.10 m to
# fly through the entire program 10 cm above the contact heights so nothing
# is physically picked or placed — useful for verifying motion + orientation
# end-to-end on real hardware before trusting the contact poses.
DRY_RUN_CLEARANCE_M = 0.0  # was 0.10 during dry run; 0.0 = full contact heights

# DRY_RUN_DISABLE_ATTACH — when True, skip the planning-scene
# attach_box_to_tcp / detach_box_at calls. Box visualisation in RViz still
# happens via the pre-spawned planning-scene boxes, but no collision body
# follows the gripper. This isolates the "attached box collides at LIFT
# config" hypothesis from the kinematic chain. Set to False for production
# (we WANT the attached-box collision check during real motion).
DRY_RUN_DISABLE_ATTACH = False  # active test: box attached -10 mm toward plane (floor)

# Independent Z offset for the box's centroid relative to the URScript TCP
# pose when attaching to rg6_tcp. NEGATIVE = box sits BELOW gripper TCP in
# world (at pick orientation). 2026-05-26: tested -5mm, didn't clear collision
# at LIFT. Larger offsets (-50mm) would clear collision but visually misplace
# the box. Real fix is touch_links, not offset. Keeping 0.0 for now.
BOX_ATTACH_Z_OFFSET_M = +0.050  # box centroid 50 mm ABOVE gripper TCP at attach. Physically, the gripper grips the SIDE of the wood block; the block extends UPWARD from the grip line. Pre-spawn box at TCP put box centroid AT TCP, so box bottom was at TCP-15mm = touching box_05 below in the planning scene. Moving the centroid +50 mm up separates the attached box from box_05 and matches physical reality better.

# WAYPOINT_TOOL_CALIBRATION_M — world-frame XYZ shift applied to every
# waypoint X/Y/Z BEFORE sending to MoveIt. Compensates for the OnRobot
# URCap's OnRobot_Single TCP not matching the actual finger grasp point
# on this cabinet (we can't edit OnRobot_Single via pendant — it's
# URCap-managed).
#
# Measured 2026-05-26 at the pick-deep pose (gripper pointing straight
# down, axis-angle (0.020, 3.133, 0.011) ~ R_y(180°)):
#   pendant TCP   = (829.66, 462.48, -375.35) mm
#   RTDE / cmd    = (823.00, 473.00,  +29.00) mm
#   pendant − cmd = (+6.66, -10.52, -404.35) mm
# The Z delta is the documented pendant-vs-RTDE 400 mm bug, NOT a real
# offset (don't compensate). The X/Y are real tool-fingertip vs URCap-TCP
# calibration error. We shift waypoints by the NEGATIVE of the delta so
# the cabinet positions the arm such that the physical grasp point lands
# at the original commanded position.
#
# IMPORTANT: only valid for waypoints with the same gripper-down orientation
# as the pick set. Place waypoints (with different wrist rotations) will
# see the SAME tool-frame calibration error manifest as a DIFFERENT world-
# frame offset. For per-pose correctness use Option 1 (set_tcp via URScript)
# or fix OnRobot_Single via the OnRobot URCap settings page if available.
WAYPOINT_TOOL_CALIBRATION_M = (-0.00666, +0.01052, +0.045)  # X/Y mm-level verified @ gripper-down. Z=+45 mm real-hw-tuned (RG6 fingers pivot down on close → command open gripper 5 mm higher than contact). Independent of attach-collision (4 tests confirmed).
if any(abs(v) > 1e-9 for v in WAYPOINT_TOOL_CALIBRATION_M):
    _wx, _wy, _wz = WAYPOINT_TOOL_CALIBRATION_M
    WAYPOINTS = {
        wp_id: (x + _wx, y + _wy, z + _wz, rx, ry, rz)
        for wp_id, (x, y, z, rx, ry, rz) in WAYPOINTS.items()
    }
if DRY_RUN_CLEARANCE_M > 0.0:
    WAYPOINTS = {
        wp_id: (x, y, z + DRY_RUN_CLEARANCE_M, rx, ry, rz)
        for wp_id, (x, y, z, rx, ry, rz) in WAYPOINTS.items()
    }

HOME_Q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]
# 2026-05-26 (later): shoulder_pan back to +pi/2 (was -pi/2) after fixing
# the ur_macro.xacro shoulder_pan_joint axis from "0 0 1" to "0 0 -1".
# URDF visualization and real cabinet now agree on this HOME — no sign-flip
# helper needed. See wiki/shoulder_pan_sign_mismatch.md.


# ---------------- Program steps ----------------
# A literal transliteration of the URScript program body.
# Each step is one of:
#   ("movej", joint_list)
#   ("movel", waypoint_index)
#   ("grip",  width_mm)
#   ("sleep", seconds)

STEPS = []
STEPS.append(("movej", HOME_Q))
STEPS.append(("sleep", 2.0))
STEPS.append(("grip", 70.0))

# 20 pick-place cycles, each: open?-move-grip-move-move-move-grip-move
# (the URScript actually has only 5 unique grip widths interleaved; we
#  honour exactly the order from the URScript program body)
GRIP_SCHEDULE = [70.0]  # initial 70 already emitted above
URSCRIPT_GRIP_PATTERN = [50.0, 60.0] * 10   # closed-then-half, repeating

# Walk the 80 waypoints in groups of 4, with grips inserted per the URScript:
#   movel WP_(4k+1) ; movel WP_(4k+2) ; sleep 2 ; grip(P[k]) ;
#   movel WP_(4k+3) ; movel WP_(4k+4)
for k in range(20):
    base = 4 * k + 1
    STEPS.append(("movel", base + 0))
    STEPS.append(("movel", base + 1))
    STEPS.append(("sleep", 2.0))
    STEPS.append(("grip", URSCRIPT_GRIP_PATTERN[k]))
    STEPS.append(("movel", base + 2))
    STEPS.append(("movel", base + 3))

# Final wrap: sleep + grip 70 + movej HOME
STEPS.append(("sleep", 2.0))
STEPS.append(("grip", 70.0))
STEPS.append(("movej", HOME_Q))


# ---------------- Helpers ----------------

def axis_angle_to_quat(rx, ry, rz):
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rx * s, ry * s, rz * s, math.cos(angle / 2.0))


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by quaternion q (q already normalised)."""
    rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
    ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
    rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz
    return rx, ry, rz


def quat_mul(a, b):
    """Hamilton product a * b for quats (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def quat_conj(q):
    """Conjugate (= inverse for unit quats)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def world_to_link_pose(tcp_world, box_world):
    """Express box_world (x,y,z,qx,qy,qz,qw) in TCP's local frame."""
    tcp_x, tcp_y, tcp_z = tcp_world[0], tcp_world[1], tcp_world[2]
    tcp_q = (tcp_world[3], tcp_world[4], tcp_world[5], tcp_world[6])
    bx, by, bz = box_world[0], box_world[1], box_world[2]
    box_q = (box_world[3], box_world[4], box_world[5], box_world[6])
    # Translation: rotate (box - tcp) by inverse of tcp rotation
    dx, dy, dz = bx - tcp_x, by - tcp_y, bz - tcp_z
    inv = quat_conj(tcp_q)
    lx, ly, lz = quat_rotate(*inv, dx, dy, dz)
    # Orientation: q_local = inv(q_tcp) * q_box
    lq = quat_mul(inv, box_q)
    return (lx, ly, lz, lq[0], lq[1], lq[2], lq[3])


def tcp_to_tool0(x_tcp, y_tcp, z_tcp, rx, ry, rz):
    """Given a URScript TCP pose, compute the tool0 pose to send to MoveIt.

    The TCP is +TCP_OFFSET_M in tool0's local Z. To put the TCP at the target,
    place tool0 at TCP - R*(0,0,TCP_OFFSET_M) where R is the goal orientation.
    """
    qx, qy, qz, qw = axis_angle_to_quat(rx, ry, rz)
    off_x, off_y, off_z = quat_rotate(qx, qy, qz, qw, 0.0, 0.0, TCP_OFFSET_M)
    return (x_tcp - off_x, y_tcp - off_y, z_tcp - off_z, qx, qy, qz, qw)


def err_name(code):
    for n in dir(MoveItErrorCodes):
        if n.isupper() and getattr(MoveItErrorCodes, n) == code:
            return n
    return f"code_{code}"


class PickPlacePlayer(Node):
    def __init__(self, real_gripper: bool = False, grip_force_n: float = RG6_DEFAULT_FORCE_N):
        super().__init__("pickplace_player")
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.real_gripper = real_gripper
        self.grip_force_n = grip_force_n
        # Always create both — cheap, and means flipping mode at runtime is trivial
        self.grip_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.urscript_pub = self.create_publisher(String, URSCRIPT_TOPIC, 10)
        # Real-hardware gripper: binary close/open via UR Tool I/O (set_io).
        # Lazily connected on first grip() call so the node still constructs
        # cleanly when the driver isn't up (sim mode, dry tests, etc.).
        self._io_grip = OnRobotToolIOGrip(self) if real_gripper else None
        self._io_grip_connected = False
        # Box visualisation via planning-scene diffs (LATCHED so move_group picks them up)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        scene_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(PlanningScene, PLANNING_SCENE_TOPIC, scene_qos)
        # Box state — id → (x_tcp, y_tcp, z_tcp, qx, qy, qz, qw) when last set in world.
        # When `attached`, the box follows rg6_tcp.
        self._boxes = {}            # box_id → "world" | "attached"
        self._next_box_id = 0

    def wait_ready(self, timeout=15.0):
        return self.mg.wait_for_server(timeout_sec=timeout)

    # ---------- joint goal (movej) ----------
    def movej(self, joint_values, planner="PTP"):
        g = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = ARM_GROUP
        r.pipeline_id = "pilz_industrial_motion_planner"
        r.planner_id = planner
        r.allowed_planning_time = PLAN_TIMEOUT
        r.max_velocity_scaling_factor = VEL_SCALE
        r.max_acceleration_scaling_factor = ACC_SCALE
        r.start_state.is_diff = True
        c = Constraints(); c.name = "j"
        for j, p in zip(UR_JOINTS, joint_values):
            jc = JointConstraint(); jc.joint_name = j; jc.position = float(p)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        g.request = r
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        return self._send(g, f"movej({planner})")

    # ---------- Cartesian goal (movel) ----------
    def movel(self, x_tcp, y_tcp, z_tcp, rx, ry, rz, planner="LIN"):
        # URScript pose is TCP-frame; transform to tool0-frame for MoveIt
        x, y, z, qx, qy, qz, qw = tcp_to_tool0(x_tcp, y_tcp, z_tcp, rx, ry, rz)

        g = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = ARM_GROUP
        r.pipeline_id = "pilz_industrial_motion_planner"
        r.planner_id = planner
        r.allowed_planning_time = PLAN_TIMEOUT
        r.max_velocity_scaling_factor = VEL_SCALE
        r.max_acceleration_scaling_factor = ACC_SCALE
        r.start_state.is_diff = True

        c = Constraints(); c.name = "cart"
        # Position constraint at the tool0 point (5 mm tolerance sphere — Pilz LIN
        # is fussy about exact endpoint; a tiny tolerance gives the IK some room)
        pc = PositionConstraint()
        pc.header.frame_id = BASE_LINK
        pc.link_name = TIP_LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        pc.constraint_region.primitives.append(sphere)
        p = Pose(); p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(p)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_LINK
        oc.link_name = TIP_LINK
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        # Loose orientation tolerance so the IK solver has wriggle room.
        # Tight orientation constraints often trigger NO_IK_SOLUTION on the
        # axis-angle poses used by the URScript program.
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.30
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        r.goal_constraints.append(c)
        g.request = r
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        return self._send(g, f"movel({planner}) TCP=({x_tcp:.3f},{y_tcp:.3f},{z_tcp:.3f})")

    # ---------- gripper ----------
    # Width threshold below which a --real-gripper grip() means CLOSE. The
    # URScript program uses ~25 mm for grip-on-block and ~80 mm for open;
    # 60 mm is a safe midpoint that maps either intent to the right line state.
    REAL_GRIP_CLOSE_THRESHOLD_MM = 60.0

    def grip(self, width_mm):
        if self.real_gripper:
            # Binary close/open via UR Tool I/O. The URScript-topic path
            # (rg_grip) is unreachable from /urscript_interface/script_command —
            # the OnRobot URCap registers its functions in a Java-backed
            # PolyScope namespace, not the URScript runtime that External
            # Control evaluates. Verified empirically 2026-05-26 via a URP
            # rebuild test (see SESSION_HANDOFF.md). Tool I/O bypasses the
            # URCap entirely: pin 16 HIGH = close, LOW = open.
            if not self._io_grip_connected:
                if not self._io_grip.connect():
                    self.get_logger().error(
                        "  grip: SetIO service unreachable — is the real-hw "
                        "driver up? Falling back to no-op for this call.")
                    return
                self._io_grip_connected = True

            close = float(width_mm) < self.REAL_GRIP_CLOSE_THRESHOLD_MM
            if close:
                ok, msg = self._io_grip.close_blocking()
                self.get_logger().info(
                    f"  grip {width_mm:.0f} mm → CLOSE (tool DO0 HIGH) — {msg}")
            else:
                ok, msg = self._io_grip.open()
                # close_blocking() already settles; open() is non-blocking,
                # so add a small settle window here for symmetry with the
                # rest of the program.
                time.sleep(GRIP_TIME)
                self.get_logger().info(
                    f"  grip {width_mm:.0f} mm → OPEN  (tool DO0 LOW)  — {msg}")
            if not ok:
                self.get_logger().warn("  grip: set_io returned success=False")
            return

        # Path 2: simulation — publish a joint trajectory using the calibrated
        # width → rad cubic (see config/rg6_width_calibration.yaml).
        target_rad = width_mm_to_angle_rad(width_mm)
        traj = JointTrajectory()
        traj.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [target_rad]
        pt.velocities = [0.0]
        sec = int(GRIP_TIME)
        nsec = int((GRIP_TIME - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.get_logger().info(
            f"  grip {width_mm:.0f} mm → rg6_joint={target_rad:.3f} rad (sim)")
        self.grip_pub.publish(traj)
        time.sleep(GRIP_TIME + 0.3)

    # ---------- box visualisation ----------
    def _publish_scene(self, ps: PlanningScene):
        # Latched-style: send a few times to ride out any subscriber races
        for _ in range(5):
            self.scene_pub.publish(ps)
            rclpy.spin_once(self, timeout_sec=0.02)

    def clear_my_boxes(self):
        """Remove any boxes from previous runs (matching box_NN ids).
        Safe to call even if the scene already has them or doesn't."""
        ps = PlanningScene(); ps.is_diff = True
        ps.robot_state.is_diff = True
        for i in range(40):  # plenty of headroom for the 10 pairs we use
            bid = f"box_{i:02d}"
            # Detach (if attached)
            aco = AttachedCollisionObject()
            aco.link_name = "rg6_tcp"; aco.object.id = bid
            aco.object.operation = CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(aco)
            # Remove world copy
            co = CollisionObject(); co.id = bid
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        self._publish_scene(ps)
        self.get_logger().info("  [box] cleared any boxes from previous runs")

    def add_box_world(self, box_id: str, x: float, y: float, z: float,
                      qx: float, qy: float, qz: float, qw: float):
        """Add a fresh 3×5×15 cm box to the planning scene at the given pose
        (TCP pose of the pick waypoint). The box's centroid sits at (x,y,z)."""
        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = BASE_LINK  # anchored to robot base so URDF base rotation moves boxes with the robot (preserves URScript-pose-to-box alignment)
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(BOX_SIZE_M)
        co.primitives.append(prim)
        p = Pose()
        p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.x = qx; p.orientation.y = qy
        p.orientation.z = qz; p.orientation.w = qw
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD

        ps = PlanningScene(); ps.is_diff = True
        ps.world.collision_objects.append(co)
        self._publish_scene(ps)
        self._boxes[box_id] = "world"
        self.get_logger().info(f"  [box] {box_id} added in world @ ({x:.3f},{y:.3f},{z:.3f})")

    def attach_box_to_tcp(self, box_id: str,
                          tcp_world, box_world):
        """Attach an existing world box to rg6_tcp at a relative pose that
        keeps the box at its current world pose (i.e. the box doesn't snap
        to the gripper centroid)."""
        rel = world_to_link_pose(tcp_world, box_world)
        rx, ry, rz, rqx, rqy, rqz, rqw = rel

        aco = AttachedCollisionObject()
        aco.link_name = "rg6_tcp"
        aco.object.id = box_id
        aco.object.operation = CollisionObject.ADD
        # Every link the box is ALLOWED to touch during transit. Without the
        # arm links + floor in this list, the planner fails because the
        # attached box (hanging 14 mm below the TCP) clips arm meshes or the
        # floor at intermediate poses.
        aco.touch_links = [
            "rg6_finger_1_finger_tip", "rg6_finger_2_finger_tip",
            "rg6_finger_1_flex_finger", "rg6_finger_2_flex_finger",
            "rg6_finger_1_truss_arm", "rg6_finger_2_truss_arm",
            "rg6_finger_1_moment_arm", "rg6_finger_2_moment_arm",
            "rg6_finger_1_origin", "rg6_finger_2_origin",
            "rg6_tcp", "rg6_body", "rg6_bracket", "ee_link",
            "wrist_1_link", "wrist_2_link", "wrist_3_link",
            "flange", "tool0", "ft_frame",
            "forearm_link", "upper_arm_link",
            "floor", "pedestal",
        ]
        prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX
        prim.dimensions = list(BOX_SIZE_M)
        aco.object.primitives.append(prim)
        p = Pose()
        p.position.x = rx; p.position.y = ry; p.position.z = rz
        p.orientation.x = rqx; p.orientation.y = rqy
        p.orientation.z = rqz; p.orientation.w = rqw
        aco.object.primitive_poses.append(p)
        aco.object.header.frame_id = "rg6_tcp"

        ps = PlanningScene(); ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        # Also remove the world copy with the same ID
        remove = CollisionObject(); remove.id = box_id
        remove.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(remove)
        self._publish_scene(ps)
        self._boxes[box_id] = "attached"
        self.get_logger().info(f"  [box] {box_id} attached to rg6_tcp (rel z={rz:.3f})")

    def detach_box_at(self, box_id: str, x: float, y: float, z: float,
                      qx: float, qy: float, qz: float, qw: float):
        """Detach a box from rg6_tcp and re-add it to world at (x,y,z,quat).
        Two-step diff: detach (REMOVE attached) + add world."""
        aco = AttachedCollisionObject()
        aco.link_name = "rg6_tcp"
        aco.object.id = box_id
        aco.object.operation = CollisionObject.REMOVE

        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = BASE_LINK  # anchored to robot base so URDF base rotation moves boxes with the robot (preserves URScript-pose-to-box alignment)
        prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX
        prim.dimensions = list(BOX_SIZE_M)
        co.primitives.append(prim)
        p = Pose()
        p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.x = qx; p.orientation.y = qy
        p.orientation.z = qz; p.orientation.w = qw
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD

        ps = PlanningScene(); ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        ps.world.collision_objects.append(co)
        self._publish_scene(ps)
        self._boxes[box_id] = "world"
        self.get_logger().info(f"  [box] {box_id} detached & placed @ ({x:.3f},{y:.3f},{z:.3f})")

    # ---------- internal ----------
    def _send(self, goal, label):
        self.get_logger().info(f"→ {label}")
        f = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=PLAN_TIMEOUT)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"  REJECTED")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=EXEC_TIMEOUT)
        r = rf.result()
        ec = r.result.error_code.val if r else -99
        if ec == 1:
            return True
        self.get_logger().warning(f"  result {err_name(ec)}")
        # Fallback: if LIN failed, retry with PTP — Pilz LIN is fussy about
        # reachability; PTP is more forgiving and still preserves the goal.
        if "LIN" in label and ec != 1:
            self.get_logger().info("  retrying as PTP")
            goal.request.planner_id = "PTP"
            f2 = self.mg.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, f2, timeout_sec=PLAN_TIMEOUT)
            gh = f2.result()
            if gh and gh.accepted:
                rf2 = gh.get_result_async()
                rclpy.spin_until_future_complete(self, rf2, timeout_sec=EXEC_TIMEOUT)
                r2 = rf2.result()
                ec2 = r2.result.error_code.val if r2 else -99
                if ec2 == 1:
                    self.get_logger().info("  PTP retry SUCCESS")
                    return True
                self.get_logger().error(f"  PTP retry also failed: {err_name(ec2)}")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max", type=int, default=None,
                    help="Stop after N pick-place cycles (default: run all 20)")
    ap.add_argument("--real-gripper", action="store_true",
                    help="Drive the real RG6 over UR Tool I/O via set_io on "
                         "pin 16 (BINARY close/open only — width is ignored). "
                         "Requires pendant Installation → Tool I/O → "
                         "Controlled by: User. Default is sim mode "
                         "(publishes JointTrajectory to the RViz mock).")
    ap.add_argument("--force", type=float, default=RG6_DEFAULT_FORCE_N,
                    help="IGNORED in --real-gripper mode (binary Tool I/O "
                         "can't set force). Kept for arg-parity with the "
                         "old URScript path.")
    args = ap.parse_args()

    rclpy.init()
    n = PickPlacePlayer(real_gripper=args.real_gripper, grip_force_n=args.force)
    if args.real_gripper:
        print("[gripper] REAL mode — binary close/open via UR Tool I/O "
              "(set_io pin 16). Width/force args ignored.")
    else:
        print("[gripper] SIM mode — publishing JointTrajectory to controller")
    if not n.wait_ready():
        print("move_action not available — is the full_stack launch running?")
        return

    # Clean up any leftover boxes from a previous run before starting fresh.
    n.clear_my_boxes()
    time.sleep(0.5)

    # Add a "pedestal" / "table" under the place positions. With the new
    # centroid-AT-pose convention, the lowest URScript place TCP is at
    # z = 0.036 → box centroid 0.036 → box BOTTOM 0.036 - 0.015 = 0.021.
    # The pedestal top sits at z = 0.020 so the bottom box rests on it
    # (1 mm clearance). Covers ONLY the place footprint so it doesn't
    # intrude into the pick approach area at Y=+0.473.
    PEDESTAL_ID = "pedestal"
    PEDESTAL_CENTRE = (0.63, -0.40, 0.0)
    PEDESTAL_SIZE = (0.25, 0.50, 0.010)        # 25 × 50 × 1 cm
    pedestal_top = 0.020
    pedestal_z = pedestal_top - PEDESTAL_SIZE[2] / 2.0
    co = CollisionObject()
    co.id = PEDESTAL_ID
    co.header.frame_id = BASE_LINK  # base_link anchored so URDF base rotation moves pedestal with the robot
    prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX
    prim.dimensions = list(PEDESTAL_SIZE)
    co.primitives.append(prim)
    p = Pose()
    p.position.x = PEDESTAL_CENTRE[0]; p.position.y = PEDESTAL_CENTRE[1]
    p.position.z = pedestal_z
    p.orientation.w = 1.0
    co.primitive_poses.append(p)
    co.operation = CollisionObject.ADD
    ps = PlanningScene(); ps.is_diff = True
    ps.world.collision_objects.append(co)
    n._publish_scene(ps)
    print(f"[scene] pedestal added at z={pedestal_top:.3f}m (so boxes rest on it)")
    time.sleep(0.3)

    # Pre-spawn ALL boxes at their PICK positions before the arm starts moving.
    # Convention: box VOLUME CENTROID sits AT the URScript TCP pick pose
    # (NOT offset by box_h/2). Pass 1 picks WP at z≈0.029 (top), pass 2 picks
    # WP at z≈-0.001 (bottom). The two stack visually with ~30 mm spacing,
    # but their centroids are exactly where the URScript program expects the
    # TCP to be at PICK time.
    PICK_WP_IDS = [2, 10, 18, 26, 34,        # pass 1: top boxes
                   42, 50, 58, 66, 74]       # pass 2: bottom boxes
    print(f"[boxes] pre-spawning {len(PICK_WP_IDS)} boxes at pick positions")
    _cx, _cy, _cz = WAYPOINT_TOOL_CALIBRATION_M
    for pair_idx, wp_id in enumerate(PICK_WP_IDS):
        wp_x, wp_y, wp_z, _, _, _ = WAYPOINTS[wp_id]
        # Centroid at the UN-shifted URScript TCP pose for this pick waypoint —
        # subtract the calibration so the box visualises where it physically
        # rests (on the pedestal / lower stack), not at the shifted gripper Z.
        n.add_box_world(f"box_{pair_idx:02d}",
                        wp_x - _cx, wp_y - _cy, wp_z - _cz,
                        0.0, 0.0, 0.0, 1.0)
    time.sleep(1.0)
    # NOTE: deliberately NOT publishing an AllowedCollisionMatrix here.
    # Publishing an ACM as a planning-scene diff REPLACES MoveIt's full ACM
    # (including SRDF-defined disable_collisions). Touch-links on attached
    # objects handle the transit-time box-vs-arm tolerance; static box-vs-arm
    # contact during approach is handled by the box sizes being smaller than
    # the gripper opening.
    print(f"[boxes] all {len(PICK_WP_IDS)} boxes visible at pick positions in RViz")

    # Box visualisation state machine.
    # Each PICK grip attaches the next box in order; each PLACE detaches it.
    last_movel_wp_id = None
    box_id_per_pair = {f"box_{i:02d}": i for i in range(len(PICK_WP_IDS))}
    pick_count = 0
    place_count = 0

    cycle = 0
    for step in STEPS:
        kind = step[0]
        if kind == "movej":
            ok = n.movej(step[1])
        elif kind == "movel":
            wp = WAYPOINTS[step[1]]
            ok = n.movel(*wp)
            last_movel_wp_id = step[1]
        elif kind == "grip":
            width = step[1]
            # Heuristic from the URScript: 50 mm = close (PICK), 60 mm = open
            # at place (PLACE), 70 mm = bookend open (no pair).
            is_close = (width <= 55.0)
            is_pair_open = (55.0 < width <= 65.0)
            is_bookend = (width > 65.0)

            # The last movel target is the pick-or-place deep pose (in URScript TCP frame).
            if last_movel_wp_id is not None and last_movel_wp_id in WAYPOINTS:
                wp_x, wp_y, wp_z, rx, ry, rz = WAYPOINTS[last_movel_wp_id]
                tcp_qx, tcp_qy, tcp_qz, tcp_qw = axis_angle_to_quat(rx, ry, rz)
            else:
                wp_x = wp_y = wp_z = 0.0
                tcp_qx = tcp_qy = tcp_qz = 0.0; tcp_qw = 1.0

            # Box pose IN WORLD: flat on the table, centroid at (wp_x, wp_y, box_h/2).
            box_world_pose = (wp_x, wp_y, BOX_HEIGHT_M / 2.0, 0.0, 0.0, 0.0, 1.0)
            tcp_world_pose = (wp_x, wp_y, wp_z, tcp_qx, tcp_qy, tcp_qz, tcp_qw)

            if is_close:
                # PICK — the box was pre-spawned with its centroid AT the
                # URScript TCP pick pose. Attach the next-in-order box.
                pair_idx = pick_count
                pick_count += 1
                bid = f"box_{pair_idx:02d}"
                box_world_at_pick = (wp_x, wp_y, wp_z + BOX_ATTACH_Z_OFFSET_M, 0.0, 0.0, 0.0, 1.0)
                n.grip(width)
                if not DRY_RUN_DISABLE_ATTACH:
                    n.attach_box_to_tcp(bid, tcp_world_pose, box_world_at_pick)
            elif is_pair_open:
                # PLACE — open gripper, detach box at the URScript place pose.
                # Convention: box CENTROID sits AT the URScript TCP release
                # pose (no box_h/2 offset). Successive places at the same XY
                # then stack at z=0.036/0.068/0.100/0.132 (URScript's design,
                # spacing slightly larger than the 30 mm box height).
                # Orientation: use the GRIPPER orientation at release so boxes
                # inherit the gripper's tilt as they were carried.
                pair_idx = place_count
                place_count += 1
                n.grip(width)
                bid = f"box_{pair_idx:02d}"
                if not DRY_RUN_DISABLE_ATTACH:
                    # SETTLE: subtract WAYPOINT_TOOL_CALIBRATION_M to place the
                    # box at its un-shifted URScript world position — i.e., where
                    # the physical block actually rests on the pedestal / stack.
                    # The calibration shift is applied to MoveIt targets (gripper
                    # TCP) to compensate the URCap calibration error, but the
                    # PHYSICAL block ends up at the original URScript coordinates.
                    _cx, _cy, _cz = WAYPOINT_TOOL_CALIBRATION_M
                    settle_x = wp_x - _cx
                    settle_y = wp_y - _cy
                    settle_z = wp_z - _cz
                    n.detach_box_at(bid,
                                    settle_x, settle_y, settle_z,
                                    tcp_qx, tcp_qy, tcp_qz, tcp_qw)
            else:
                # Bookend grip (initial or final open) — no box pair
                n.grip(width)

            ok = True
            cycle += 1
        elif kind == "sleep":
            time.sleep(step[1])
            ok = True
        else:
            ok = True

        if not ok:
            print(f"\nStopping at step {step} due to failure.")
            break

        if args.max is not None and cycle > args.max:
            print(f"\nReached --max {args.max} cycles, stopping.")
            break

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Minimal real-hardware smoke test — UR10e Z up/down around HOME + RG6 cycle.

What it does (per cycle, default 1 cycle):
  1. movej → HOME (the verified safe joint config)
  2. movej → HOME with shoulder_lift NUDGED MORE NEGATIVE (TCP moves UP)
  3. grip → CLOSE_WIDTH_MM
  4. movej → HOME
  5. movej → HOME with shoulder_lift NUDGED LESS NEGATIVE (TCP moves DOWN)
  6. grip → OPEN_WIDTH_MM
  7. movej → HOME (return)

Why joint-space, not Cartesian: at the verified HOME the arm is in a
clean non-singular configuration; a ±0.05 rad nudge on shoulder_lift
produces ≈ ±3 cm TCP-Z motion with the arm staying close to HOME. No
IK surprises, no MoveIt Cartesian planner edge cases.

SAFETY by design:
  - VEL_SCALE / ACC_SCALE = 0.05 (5% — very slow for first real-HW run)
  - Joint perturbation capped at MAX_DELTA_RAD (0.10 rad ≈ 6 cm TCP)
  - Default --delta-rad = 0.05 rad (≈ 3 cm)
  - --yes is REQUIRED to actually move. Without it, this is a pure
    dry-run that prints the joint vectors and gripper widths it would
    have commanded but sends nothing.
  - Gripper default force 25 N (well below 120 N max).

Usage:
    # Pure dry run — prints intent, sends nothing
    python3 real_hw_smoke.py

    # Sim arm (mock controller) + sim gripper trajectory
    python3 real_hw_smoke.py --yes

    # Real arm + sim gripper (no URCap call — useful as the FIRST real-HW test)
    python3 real_hw_smoke.py --yes

    # Real arm + REAL gripper (URScript rg_grip() via URCap)
    python3 real_hw_smoke.py --yes --real-gripper --force 25

    # 3 cycles at 4 cm nudge
    python3 real_hw_smoke.py --yes --cycles 3 --delta-rad 0.07
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             MoveItErrorCodes)
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ---------------- Verified safe config ----------------

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ARM_GROUP = "ur_manipulator"

# THE user-verified HOME. Same vector used in SRDF home group_state,
# initial_positions.yaml, and play_pickplace.py HOME_Q. Do not change.
HOME_Q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]

# Hard caps. Even if --delta-rad is set higher, the script refuses to
# exceed these. Each cap is intentionally below the joint limits with
# huge margin so a typo can't drive the arm into a singularity.
MAX_DELTA_RAD = 0.10           # ≈ 6 cm TCP-Z motion — cap regardless of CLI
                               # (UR10e upper-arm length 612 mm × sin(0.10) ≈ 61 mm)
DEFAULT_DELTA_RAD = 0.05       # ≈ 3 cm TCP-Z motion — the sensible default

# Conservative speeds for real hardware. Memory rule: always default
# to safe-slow on the UR; only ramp on explicit ask.
VEL_SCALE = 0.05
ACC_SCALE = 0.05

PLAN_TIMEOUT = 10.0
EXEC_TIMEOUT = 60.0

# Gripper widths used by the smoke test. Default OPEN < full open (153 mm)
# so even a calibration drift doesn't drive the fingers into the body.
DEFAULT_OPEN_WIDTH_MM = 100.0
DEFAULT_CLOSE_WIDTH_MM = 80.0
GRIPPER_HOLD_S = 1.5

GRIPPER_TOPIC = "/rg6_gripper_controller/joint_trajectory"
GRIPPER_JOINT = "rg6_joint"
URSCRIPT_TOPIC = "/urscript_interface/script_command"


# Width-mm → angle-rad cubic from the calibration yaml. Copy of the
# tiny helper in play_pickplace.py so this file stays standalone.
import os
import yaml

_CALIB_PATH = os.path.expanduser(
    "~/ur_rg6_ws/install/ur10e_rg6_moveit_config/share/"
    "ur10e_rg6_moveit_config/config/rg6_width_calibration.yaml")
try:
    with open(_CALIB_PATH) as _f:
        _C = yaml.safe_load(_f)["rg6_joint_to_width_mm"]
    _COEFF = _C["cubic_angle_of_width"]
    _MIN_W, _MAX_W = _C["min_width_mm"], _C["max_width_mm"]
except Exception:
    _COEFF = [-2.6099e-07, 4.1813e-05, -0.008620, 1.29513]
    _MIN_W, _MAX_W = 1.16, 153.17


def width_mm_to_angle_rad(width_mm: float) -> float:
    w = max(_MIN_W, min(_MAX_W, float(width_mm)))
    a3, a2, a1, a0 = _COEFF
    rad = ((a3 * w + a2) * w + a1) * w + a0
    return max(0.0, min(1.30, rad))


def err_name(code):
    for n in dir(MoveItErrorCodes):
        if n.isupper() and getattr(MoveItErrorCodes, n) == code:
            return n
    return f"code_{code}"


# ---------------- Node ----------------

class RealHwSmoke(Node):
    def __init__(self, real_gripper: bool, force_n: float, dry_run: bool):
        super().__init__("real_hw_smoke")
        self.real_gripper = real_gripper
        self.force_n = force_n
        self.dry_run = dry_run
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.grip_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.urs_pub = self.create_publisher(String, URSCRIPT_TOPIC, 10)

    # ----- arm -----
    def movej(self, joint_values, label):
        # Hard safety: every commanded joint must be within MAX_DELTA_RAD
        # of HOME. Refuse if any single joint exceeds the cap.
        for i, (q, q_home) in enumerate(zip(joint_values, HOME_Q)):
            if abs(q - q_home) > MAX_DELTA_RAD + 1e-9:
                msg = (f"REFUSED: joint {UR_JOINTS[i]} would deviate "
                       f"{abs(q - q_home):.3f} rad from HOME "
                       f"(cap {MAX_DELTA_RAD:.3f}).")
                self.get_logger().error(msg)
                return False

        vec = "[" + ", ".join(f"{v:+.4f}" for v in joint_values) + "]"
        self.get_logger().info(f"→ {label:>16} q = {vec}")

        if self.dry_run:
            self.get_logger().info(f"  (dry-run, not sending)")
            return True

        if not self.mg.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/move_action not available")
            return False
        g = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = ARM_GROUP
        # OMPL — more forgiving than Pilz PTP for tiny moves
        r.pipeline_id = "ompl"
        r.planner_id = "RRTConnectkConfigDefault"
        r.allowed_planning_time = PLAN_TIMEOUT
        r.max_velocity_scaling_factor = VEL_SCALE
        r.max_acceleration_scaling_factor = ACC_SCALE
        r.start_state.is_diff = True

        c = Constraints(); c.name = "j"
        for j, v in zip(UR_JOINTS, joint_values):
            jc = JointConstraint()
            jc.joint_name = j; jc.position = float(v)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        g.request = r
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True

        f = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=PLAN_TIMEOUT)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"  {label} REJECTED at plan stage")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=EXEC_TIMEOUT)
        res = rf.result()
        ec = res.result.error_code.val if res else -99
        if ec != 1:
            self.get_logger().warning(f"  {label} {err_name(ec)}")
            return False
        return True

    # ----- gripper -----
    def grip(self, width_mm: float, hold_s: float = GRIPPER_HOLD_S):
        if self.real_gripper:
            cmd = (f"rg_grip({float(width_mm):.1f}, {float(self.force_n):.1f}, "
                   f"tool_index=0, blocking=True, depth_comp=False, popupmsg=False)\n")
            self.get_logger().info(
                f"  grip {width_mm:>5.1f} mm @ {self.force_n:.0f} N  [URScript]")
            if self.dry_run:
                self.get_logger().info(f"  (dry-run, URScript NOT published)")
                return
            self.urs_pub.publish(String(data=cmd))
        else:
            rad = width_mm_to_angle_rad(width_mm)
            self.get_logger().info(
                f"  grip {width_mm:>5.1f} mm → rg6_joint = {rad:.3f} rad  [sim]")
            if self.dry_run:
                self.get_logger().info(f"  (dry-run, JointTrajectory NOT published)")
                return
            t = JointTrajectory()
            t.joint_names = [GRIPPER_JOINT]
            p = JointTrajectoryPoint()
            p.positions = [rad]; p.velocities = [0.0]
            p.time_from_start = Duration(sec=int(hold_s),
                                         nanosec=int((hold_s % 1) * 1e9))
            t.points.append(p)
            self.grip_pub.publish(t)
        time.sleep(hold_s + 0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true",
                    help="Actually move. Without this, the script is a "
                         "pure dry-run (prints intent only).")
    ap.add_argument("--real-gripper", action="store_true",
                    help="Publish URScript rg_grip() via the OnRobot URCap "
                         "(real hardware). Default is sim JointTrajectory.")
    ap.add_argument("--force", type=float, default=25.0,
                    help="Grip force in N for --real-gripper mode (default 25 — "
                         "well below the 120 N max).")
    ap.add_argument("--cycles", type=int, default=1,
                    help="How many full up/grip/home/down/grip/home cycles to run "
                         "(default 1).")
    ap.add_argument("--delta-rad", type=float, default=DEFAULT_DELTA_RAD,
                    help=f"shoulder_lift perturbation in radians "
                         f"(default {DEFAULT_DELTA_RAD}, capped at "
                         f"{MAX_DELTA_RAD}; ≈ {DEFAULT_DELTA_RAD*60:.0f} mm TCP-Z).")
    ap.add_argument("--open-mm", type=float, default=DEFAULT_OPEN_WIDTH_MM)
    ap.add_argument("--close-mm", type=float, default=DEFAULT_CLOSE_WIDTH_MM)
    args = ap.parse_args()

    # Hard cap on the perturbation
    if args.delta_rad > MAX_DELTA_RAD:
        print(f"--delta-rad {args.delta_rad} > MAX_DELTA_RAD {MAX_DELTA_RAD}; "
              f"clamping.")
        args.delta_rad = MAX_DELTA_RAD
    if args.delta_rad <= 0:
        print(f"--delta-rad must be > 0; got {args.delta_rad}.")
        return

    # Up = MORE NEGATIVE shoulder_lift (tilts upper arm back, TCP rises
    # at this base/elbow config). Down = LESS NEGATIVE.
    UP_Q   = HOME_Q.copy(); UP_Q[1]   = HOME_Q[1] - args.delta_rad
    DOWN_Q = HOME_Q.copy(); DOWN_Q[1] = HOME_Q[1] + args.delta_rad

    dry_run = not args.yes

    print(f"[mode]    arm = {'DRY-RUN' if dry_run else 'LIVE'}  "
          f"gripper = {'REAL (URScript)' if args.real_gripper else 'SIM (JointTrajectory)'}")
    print(f"[speeds]  vel = {VEL_SCALE:.2f}  acc = {ACC_SCALE:.2f}")
    print(f"[motion]  delta = {args.delta_rad:.3f} rad "
          f"(≈ {args.delta_rad*612:.0f} mm TCP-Z); cycles = {args.cycles}")
    print(f"[grip]    open = {args.open_mm:.0f} mm, "
          f"close = {args.close_mm:.0f} mm, force = {args.force:.0f} N")
    print(f"[HOME]    {HOME_Q}")
    print(f"[UP_Q]    {UP_Q}")
    print(f"[DOWN_Q]  {DOWN_Q}")
    print()

    if dry_run:
        print("DRY-RUN — no motion will be commanded. Re-run with --yes to execute.\n")

    rclpy.init()
    n = RealHwSmoke(real_gripper=args.real_gripper,
                    force_n=args.force, dry_run=dry_run)

    try:
        # 0. Get to HOME first (slow, single big move). Skip if dry-run.
        ok = n.movej(HOME_Q, "HOME (start)")
        if not ok and not dry_run:
            print("Could not reach HOME — aborting.")
            return
        n.grip(args.open_mm)

        for i in range(args.cycles):
            print(f"\n--- cycle {i+1}/{args.cycles} ---")
            if not n.movej(UP_Q, "UP"):     break
            n.grip(args.close_mm)
            if not n.movej(HOME_Q, "HOME"): break
            if not n.movej(DOWN_Q, "DOWN"): break
            n.grip(args.open_mm)
            if not n.movej(HOME_Q, "HOME"): break

        # Final: HOME with gripper open at a safe boot-like width.
        print()
        n.movej(HOME_Q, "HOME (end)")
        n.grip(args.open_mm)
        print("\n[done]")

    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

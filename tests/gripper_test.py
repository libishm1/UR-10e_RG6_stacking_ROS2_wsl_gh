"""Minimal gripper + small arm motion test.

The arm goes to HOME, then oscillates between a slightly-UP and slightly-DOWN
joint configuration close to home (~7 cm vertical swing) while the gripper
cycles through widths. No MoveIt LIN/Cartesian goals — just Pilz PTP joint
goals on ur_manipulator. Safe, predictable, no IK headaches.

Two gripper modes (same as play_pickplace.py):
  default      → JointTrajectory on /rg6_gripper_controller/joint_trajectory
                 Uses the calibrated width-mm → rg6_joint-rad cubic.
  --real       → URScript rg_grip(width_mm, force_N) on
                 /urscript_interface/script_command. OnRobot URCap on the
                 UR pendant executes it. No Compute Box needed.

Usage:
    python3 gripper_test.py                              # default sweep
    python3 gripper_test.py --no-arm                     # gripper only
    python3 gripper_test.py --widths 70 50 60 --hold 2
    python3 gripper_test.py --real --force 30
"""
import argparse
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             MoveItErrorCodes)


# ---------------- Constants ----------------

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ARM_GROUP = "ur_manipulator"

# Safe poses close to home.  HOME has the arm folded inward with the wrist
# tucked under — a known reachable, collision-free configuration.
HOME_Q = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
# UP/DOWN swing shoulder_lift by ±0.10 rad ≈ ±6 cm vertical, everything else
# unchanged.  Stays well within the work envelope.
UP_Q   = [0.0, -1.6708, 0.0, -1.5708, 0.0, 0.0]
DOWN_Q = [0.0, -1.4708, 0.0, -1.5708, 0.0, 0.0]

VEL_SCALE = 0.20      # MoveIt velocity scaling — conservative
ACC_SCALE = 0.20
PLAN_TIMEOUT = 5.0
EXEC_TIMEOUT = 30.0

GRIPPER_TOPIC = "/rg6_gripper_controller/joint_trajectory"
URSCRIPT_TOPIC = "/urscript_interface/script_command"
GRIPPER_JOINT = "rg6_joint"


# ---------------- Width → angle calibration ----------------

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


# ---------------- The node ----------------

class GripperArmTest(Node):
    def __init__(self, real: bool, force_n: float, use_arm: bool):
        super().__init__("gripper_test")
        self.real = real
        self.force_n = force_n
        self.use_arm = use_arm
        self.traj_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.urs_pub = self.create_publisher(String, URSCRIPT_TOPIC, 10)
        if use_arm:
            self.mg = ActionClient(self, MoveGroup, "/move_action")
        else:
            self.mg = None

    # ----- gripper -----
    def grip(self, width_mm: float, hold_s: float = 1.5):
        if self.real:
            cmd = (f"rg_grip({float(width_mm):.1f}, {float(self.force_n):.1f}, "
                   f"tool_index=0, blocking=True, depth_comp=False, popupmsg=False)\n")
            self.get_logger().info(
                f"  grip {width_mm:>6.1f} mm @ {self.force_n:.0f} N  [URScript]")
            self.urs_pub.publish(String(data=cmd))
        else:
            rad = width_mm_to_angle_rad(width_mm)
            self.get_logger().info(
                f"  grip {width_mm:>6.1f} mm → rg6_joint = {rad:.3f} rad  [sim]")
            t = JointTrajectory()
            t.joint_names = [GRIPPER_JOINT]
            p = JointTrajectoryPoint()
            p.positions = [rad]
            p.velocities = [0.0]
            p.time_from_start = Duration(sec=int(hold_s), nanosec=int((hold_s % 1) * 1e9))
            t.points.append(p)
            self.traj_pub.publish(t)
        time.sleep(hold_s + 0.3)

    # ----- arm (Pilz PTP joint goal) -----
    def movej(self, joint_values, label):
        if self.mg is None:
            return True
        if not self.mg.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_action not available")
            return False
        g = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = ARM_GROUP
        # OMPL is more forgiving about start-state edge cases than Pilz PTP
        # (which checks workspace bounds + start-validity tightly). For a
        # tiny ±6 cm joint move we don't need Pilz's time-optimal profile.
        r.pipeline_id = "ompl"
        r.planner_id = "RRTConnectkConfigDefault"
        r.allowed_planning_time = PLAN_TIMEOUT
        r.max_velocity_scaling_factor = VEL_SCALE
        r.max_acceleration_scaling_factor = ACC_SCALE
        r.start_state.is_diff = True
        c = Constraints(); c.name = "j"
        for j, v in zip(UR_JOINTS, joint_values):
            jc = JointConstraint(); jc.joint_name = j; jc.position = float(v)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        g.request = r
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(f"→ arm {label}")
        f = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=PLAN_TIMEOUT)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"  {label} rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=EXEC_TIMEOUT)
        res = rf.result()
        ec = res.result.error_code.val if res else -99
        ok = ec == 1
        if not ok:
            self.get_logger().warning(f"  {label} error_code={ec}")
        return ok


# ---------------- Sequence ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--widths", type=float, nargs="+",
                    default=[70.0, 50.0, 60.0, 70.0],
                    help="Sequence of widths to command between arm moves")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="Seconds to hold each gripper width (default 2)")
    ap.add_argument("--no-arm", action="store_true",
                    help="Gripper only — skip the arm up/down moves")
    ap.add_argument("--real", action="store_true",
                    help="Real-gripper mode (URScript rg_grip())")
    ap.add_argument("--force", type=float, default=40.0,
                    help="Grip force in N for --real (default 40)")
    args = ap.parse_args()

    rclpy.init()
    n = GripperArmTest(real=args.real, force_n=args.force, use_arm=not args.no_arm)
    print(f"[gripper] {'REAL (URScript)' if args.real else 'SIM (JointTrajectory)'}")
    print(f"[arm]     {'no-arm' if args.no_arm else 'enabled (HOME ↔ UP ↔ DOWN, ±6 cm)'}")
    print(f"[widths]  {args.widths}    [hold] {args.hold} s")
    time.sleep(0.5)

    if args.no_arm:
        # Just sweep widths
        for w in args.widths:
            n.grip(w, args.hold)
    else:
        # 1. Start at home
        if not n.movej(HOME_Q, "HOME"):
            print("Couldn't reach HOME — aborting.")
            n.destroy_node(); rclpy.shutdown(); return
        n.grip(args.widths[0], args.hold)

        # 2. Alternate UP / DOWN with each width
        for i, w in enumerate(args.widths[1:], start=1):
            target = UP_Q if (i % 2 == 1) else DOWN_Q
            label = "UP" if (i % 2 == 1) else "DOWN"
            if not n.movej(target, label):
                print(f"Couldn't reach {label} — aborting.")
                break
            n.grip(w, args.hold)

        # 3. Back to home, gripper at safe boot
        n.movej(HOME_Q, "HOME (return)")
        n.grip(70.0, args.hold)

    print("[done]")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

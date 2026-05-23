"""Verify Pilz (PTP) plan+execute on every group."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, PlanningOptions, Constraints,
                             JointConstraint)


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def plan_exec(node, mg, group, joint_targets, planner, label):
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1
    req.pipeline_id = "pilz_industrial_motion_planner"
    req.planner_id = planner
    req.start_state.is_diff = True

    c = Constraints(); c.name = "goal"
    for j, p in joint_targets.items():
        jc = JointConstraint()
        jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    goal.request = req
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    f = mg.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    gh = f.result()
    if not gh or not gh.accepted:
        print(f"  [{label}] REJECTED"); return False
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60.0)
    r = rf.result()
    ec = r.result.error_code.val if r else None
    print(f"  [{label}] error_code={ec} ({'SUCCESS' if ec==1 else 'FAIL'})")
    return ec == 1


def main():
    rclpy.init()
    n = Node("pilz_group_test")
    mg = ActionClient(n, MoveGroup, "/move_action")
    if not mg.wait_for_server(timeout_sec=10.0):
        print("FAIL: move_action missing"); return

    home = dict(zip(UR_JOINTS, [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]))
    away = dict(zip(UR_JOINTS, [0.4, -1.3, 0.9, -1.4, 0.2, 0.0]))

    print("Pilz PTP — ur_manipulator")
    a = plan_exec(n, mg, "ur_manipulator", away, "PTP", "ur_manipulator")
    print("Pilz PTP — rg6_gripper")
    b = plan_exec(n, mg, "rg6_gripper", {"rg6_joint": 0.1}, "PTP", "rg6_gripper")
    print("Pilz PTP — arm_with_gripper")
    combined = dict(home); combined["rg6_joint"] = 0.5
    c = plan_exec(n, mg, "arm_with_gripper", combined, "PTP", "arm_with_gripper")

    n.destroy_node(); rclpy.shutdown()
    print()
    print("=" * 40)
    print(f"  ur_manipulator    : {'PASS' if a else 'FAIL'}")
    print(f"  rg6_gripper       : {'PASS' if b else 'FAIL'}")
    print(f"  arm_with_gripper  : {'PASS' if c else 'FAIL'}")


if __name__ == "__main__":
    main()

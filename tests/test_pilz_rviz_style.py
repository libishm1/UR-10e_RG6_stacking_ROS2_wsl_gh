"""Reproduce the EXACT RViz failure mode: Pilz PTP on arm_with_gripper
with goal constraints ONLY for the 6 UR joints (no rg6_joint specified).
Then also test LIN. This is what RViz actually sends when you use the
interactive marker on the arm and click Plan & Execute.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.msg import MoveItErrorCodes

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def err(v):
    for n in dir(MoveItErrorCodes):
        if n.isupper() and getattr(MoveItErrorCodes, n) == v:
            return n
    return str(v)


def run(node, mg, group, ur_target, planner, include_grip):
    g = MoveGroup.Goal()
    r = MotionPlanRequest()
    r.group_name = group
    r.pipeline_id = "pilz_industrial_motion_planner"
    r.planner_id = planner
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.1
    r.max_acceleration_scaling_factor = 0.1
    r.start_state.is_diff = True
    c = Constraints(); c.name = "g"
    for j, p in zip(UR_JOINTS, ur_target):
        jc = JointConstraint(); jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    if include_grip:
        jc = JointConstraint(); jc.joint_name = "rg6_joint"; jc.position = 0.3
        jc.tolerance_above = 0.05; jc.tolerance_below = 0.05; jc.weight = 1.0
        c.joint_constraints.append(jc)
    r.goal_constraints.append(c)
    g.request = r
    g.planning_options.planning_scene_diff.is_diff = True
    g.planning_options.planning_scene_diff.robot_state.is_diff = True

    f = mg.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5)
    gh = f.result()
    if not gh or not gh.accepted:
        return ("REJECTED", -1)
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30)
    res = rf.result()
    ec = res.result.error_code.val if res else None
    return (err(ec), ec)


def main():
    rclpy.init()
    n = Node("pilz_rviz_style")
    mg = ActionClient(n, MoveGroup, "/move_action")
    mg.wait_for_server(timeout_sec=10)

    ur_a = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    ur_b = [0.5, -1.3, 0.9, -1.4, 0.2, 0.0]

    print("--- PTP on arm_with_gripper, rg6_joint NOT in goal (RViz style) ---")
    print("  attempt 1:", run(n, mg, "arm_with_gripper", ur_a, "PTP", include_grip=False))
    print("  attempt 2:", run(n, mg, "arm_with_gripper", ur_b, "PTP", include_grip=False))
    print("  attempt 3:", run(n, mg, "arm_with_gripper", ur_a, "PTP", include_grip=False))

    print()
    print("--- PTP on arm_with_gripper, rg6_joint INCLUDED in goal (control) ---")
    print("  attempt 1:", run(n, mg, "arm_with_gripper", ur_b, "PTP", include_grip=True))
    print("  attempt 2:", run(n, mg, "arm_with_gripper", ur_a, "PTP", include_grip=True))

    print()
    print("--- LIN on arm_with_gripper, rg6_joint NOT in goal ---")
    print("  attempt 1:", run(n, mg, "arm_with_gripper", ur_b, "LIN", include_grip=False))
    print("  attempt 2:", run(n, mg, "arm_with_gripper", ur_a, "LIN", include_grip=False))

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

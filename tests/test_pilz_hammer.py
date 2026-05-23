"""Hammer Pilz PTP arm_with_gripper with 20 sequential goals — varied:
no gripper goal, gripper at various positions, Cartesian-ish poses via joint
constraints, oscillating between large and small arm motions. If any fail,
print the move_group's log tail for that one."""
import rclpy
import time
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

UR_JOINTS = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
             "wrist_1_joint","wrist_2_joint","wrist_3_joint"]


def make_goal(ur_target, rg6_target=None):
    g = MoveGroup.Goal()
    r = MotionPlanRequest()
    r.group_name = "arm_with_gripper"
    r.pipeline_id = "pilz_industrial_motion_planner"
    r.planner_id = "PTP"
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.1
    r.max_acceleration_scaling_factor = 0.1
    r.start_state.is_diff = True
    c = Constraints(); c.name = "g"
    for j, p in zip(UR_JOINTS, ur_target):
        jc = JointConstraint(); jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    if rg6_target is not None:
        jc = JointConstraint(); jc.joint_name = "rg6_joint"; jc.position = rg6_target
        jc.tolerance_above = 0.05; jc.tolerance_below = 0.05; jc.weight = 1.0
        c.joint_constraints.append(jc)
    r.goal_constraints.append(c)
    g.request = r
    g.planning_options.planning_scene_diff.is_diff = True
    g.planning_options.planning_scene_diff.robot_state.is_diff = True
    return g


def main():
    rclpy.init()
    n = Node("pilz_hammer")
    mg = ActionClient(n, MoveGroup, "/move_action")
    mg.wait_for_server(timeout_sec=10)

    home   = [0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0]
    poses = [
        ([0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0], None),
        ([0.5,  -1.3,    0.9,  -1.4,    0.2, 0.0], 0.3),
        ([-0.5, -1.4,    0.8,  -1.3,   -0.2, 0.0], None),
        ([0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0], 0.5),
        ([0.3,  -1.2,    1.1,  -1.4,    0.1, 0.0], None),
        ([-0.3, -1.4,    0.6,  -1.5,    0.3, 0.0], 0.0),
        ([0.0,  -1.5708, 0.5,  -1.5708, 0.0, 0.5], None),
        ([0.7,  -1.0,    1.2,  -1.3,    0.5, 0.5], 1.0),
        ([0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0], None),
        ([-0.7, -1.0,    1.2,  -1.3,   -0.5, -0.5], 0.2),
    ]
    # repeat twice
    poses = poses + poses

    successes = 0
    failures = []
    for i, (ur, rg6) in enumerate(poses, start=1):
        g = make_goal(ur, rg6)
        f = mg.send_goal_async(g)
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        gh = f.result()
        if not gh or not gh.accepted:
            failures.append((i, "REJECTED", ur, rg6))
            continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        r = rf.result()
        ec = r.result.error_code.val if r else -99
        if ec == 1:
            successes += 1
            print(f"  goal {i:2d}: SUCCESS (rg6={rg6})")
        else:
            failures.append((i, f"ec={ec}", ur, rg6))
            print(f"  goal {i:2d}: FAIL ec={ec} (rg6={rg6})")
        time.sleep(0.3)

    n.destroy_node(); rclpy.shutdown()
    print(f"\nTotal: {successes}/{len(poses)} passed")
    if failures:
        print("\nFailures:")
        for i, why, ur, rg6 in failures:
            print(f"  {i}: {why}  ur={ur} rg6={rg6}")


if __name__ == "__main__":
    main()

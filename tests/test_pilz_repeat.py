"""Run 5 Pilz PTP plans back-to-back to reproduce the flakiness."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.msg import MoveItErrorCodes

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

POSES = [
    [0.0,  -1.5708,  0.0,  -1.5708, 0.0, 0.0],
    [0.5,  -1.3,     0.9,  -1.4,    0.2, 0.0],
    [-0.5, -1.4,     0.8,  -1.3,   -0.2, 0.0],
    [0.0,  -1.5708,  0.0,  -1.5708, 0.0, 0.0],
    [0.3,  -1.2,     1.1,  -1.4,    0.1, 0.0],
]

def err_name(v):
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == v:
            return name
    return f"code_{v}"


def run(node, mg, target, label):
    goal = MoveGroup.Goal()
    r = MotionPlanRequest()
    r.group_name = "arm_with_gripper"
    r.pipeline_id = "pilz_industrial_motion_planner"
    r.planner_id = "PTP"
    r.num_planning_attempts = 1
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.1
    r.max_acceleration_scaling_factor = 0.1
    r.start_state.is_diff = True
    c = Constraints(); c.name = "g"
    for j, p in zip(UR_JOINTS, target):
        jc = JointConstraint(); jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    # Always also specify rg6_joint so the gripper has motion to plan; otherwise
    # the JTC rejects the empty/no-motion trajectory part.
    jc = JointConstraint(); jc.joint_name = "rg6_joint"
    jc.position = 0.2 + 0.1 * (hash(label) % 5)  # vary each call
    jc.tolerance_above = 0.05; jc.tolerance_below = 0.05; jc.weight = 1.0
    c.joint_constraints.append(jc)
    r.goal_constraints.append(c)
    goal.request = r
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    f = mg.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    gh = f.result()
    if not gh or not gh.accepted:
        print(f"  {label}: REJECTED"); return False
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
    r2 = rf.result()
    ec = r2.result.error_code.val if r2 else None
    print(f"  {label}: {err_name(ec)} ({ec})")
    return ec == 1


def main():
    rclpy.init()
    n = Node("pilz_repeat")
    mg = ActionClient(n, MoveGroup, "/move_action")
    if not mg.wait_for_server(timeout_sec=10.0):
        print("FAIL: no move_action"); return
    results = []
    for i, p in enumerate(POSES):
        results.append(run(n, mg, p, f"goal {i+1}"))
        import time; time.sleep(0.5)
    n.destroy_node(); rclpy.shutdown()
    print()
    print(f"  total: {sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()

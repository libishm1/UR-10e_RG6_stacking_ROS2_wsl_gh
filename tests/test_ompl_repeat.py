"""Same 5 goals as test_pilz_repeat, but via OMPL — sanity that OMPL is
reliable on arm_with_gripper even when the gripper barely moves."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
import time

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

POSES = [
    [0.0,  -1.5708,  0.0,  -1.5708, 0.0, 0.0],
    [0.5,  -1.3,     0.9,  -1.4,    0.2, 0.0],
    [-0.5, -1.4,     0.8,  -1.3,   -0.2, 0.0],
    [0.0,  -1.5708,  0.0,  -1.5708, 0.0, 0.0],
    [0.3,  -1.2,     1.1,  -1.4,    0.1, 0.0],
]


def run(node, mg, target, label):
    g = MoveGroup.Goal()
    r = MotionPlanRequest()
    r.group_name = "arm_with_gripper"
    r.pipeline_id = "ompl"
    r.planner_id = "RRTConnectkConfigDefault"
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.1
    r.max_acceleration_scaling_factor = 0.1
    r.start_state.is_diff = True
    c = Constraints(); c.name = "g"
    for j, p in zip(UR_JOINTS, target):
        jc = JointConstraint(); jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    r.goal_constraints.append(c)
    g.request = r
    g.planning_options.planning_scene_diff.is_diff = True
    g.planning_options.planning_scene_diff.robot_state.is_diff = True

    f = mg.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    gh = f.result()
    if not gh or not gh.accepted:
        print(f"  {label}: REJECTED"); return False
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
    r2 = rf.result()
    ec = r2.result.error_code.val if r2 else None
    print(f"  {label}: error_code={ec}")
    return ec == 1


def main():
    rclpy.init()
    n = Node("ompl_repeat")
    mg = ActionClient(n, MoveGroup, "/move_action")
    mg.wait_for_server(timeout_sec=10)
    results = []
    for i, p in enumerate(POSES):
        results.append(run(n, mg, p, f"goal {i+1}"))
        time.sleep(0.5)
    n.destroy_node(); rclpy.shutdown()
    print(f"  total: {sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()

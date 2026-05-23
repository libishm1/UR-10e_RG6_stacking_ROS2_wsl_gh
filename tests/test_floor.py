"""Verify the floor link blocks below-z=0 plans.

We pick an arm pose that would put tool0 BELOW z=0 (it'd dive through the floor),
then ask MoveIt to plan to it via ur_manipulator. If the floor is registered as
a collision, the plan should fail with a collision/IK error. Without the floor,
it would have succeeded.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             RobotState, PositionIKRequest)
from moveit_msgs.srv import GetStateValidity, GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Tester(Node):
    def __init__(self):
        super().__init__("floor_test")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)


def main():
    rclpy.init()
    n = Tester()
    # 1. wait for joint state
    import time
    deadline = time.time() + 5
    while n.js is None and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.2)

    # 2. Configuration that would dive through the floor
    #    shoulder_lift very low, elbow forward, wrist down — tool below z=0
    below_floor = [0.0, -0.2, 1.5, -1.5, 0.0, 0.0]

    print("Test A: check_state_validity on the below-floor joint state")
    cli = n.create_client(GetStateValidity, "/check_state_validity")
    cli.wait_for_service(timeout_sec=5)
    req = GetStateValidity.Request()
    req.group_name = "ur_manipulator"
    rs = RobotState()
    rs.joint_state.name = UR_JOINTS
    rs.joint_state.position = below_floor
    req.robot_state = rs
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    r = f.result()
    print(f"  state valid: {r.valid}")
    if r.contacts:
        for c in r.contacts[:3]:
            print(f"  contact: {c.contact_body_1} ↔ {c.contact_body_2}")

    print()
    print("Test B: ask MoveIt to plan to that pose (expect failure)")
    mg = ActionClient(n, MoveGroup, "/move_action")
    mg.wait_for_server(timeout_sec=10)
    g = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = "ur_manipulator"
    req.pipeline_id = "ompl"
    req.planner_id = "RRTConnectkConfigDefault"
    req.allowed_planning_time = 3.0
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1
    req.start_state.is_diff = True
    c = Constraints(); c.name = "below"
    for j, p in zip(UR_JOINTS, below_floor):
        jc = JointConstraint(); jc.joint_name = j; jc.position = p
        jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    g.request = req
    g.planning_options.plan_only = True
    g.planning_options.planning_scene_diff.is_diff = True
    g.planning_options.planning_scene_diff.robot_state.is_diff = True

    f = mg.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    gh = f.result()
    if not gh.accepted:
        print("  rejected")
    else:
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        r2 = rf.result()
        ec = r2.result.error_code.val if r2 else None
        print(f"  error_code={ec} (1=SUCCESS, anything else=blocked)")
        if ec == 1:
            print("  ⚠️ plan succeeded — floor NOT blocking")
        else:
            print("  ✅ plan blocked by floor")

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

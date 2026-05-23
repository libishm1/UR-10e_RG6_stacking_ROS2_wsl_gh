"""Headless MoveIt 2 smoke tests:
  1. Call /compute_ik on a known reachable pose. Expect SUCCESS.
  2. Send a MoveGroup goal to a slightly different joint config.
     Expect plan SUCCESS, execution SUCCESS, RViz arm visibly moves.

Speeds are kept conservative on purpose (vel_scale=0.2, planning_time=10 s).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, JointConstraint, Constraints, MotionPlanRequest, PlanningOptions, PositionIKRequest
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class MoveItTester(Node):
    def __init__(self):
        super().__init__("moveit_smoke_tester")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.mg = ActionClient(self, MoveGroup, "/move_action")

    def test_ik(self) -> bool:
        self.get_logger().info("Test 1: /compute_ik")
        if not self.ik_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("compute_ik service not available")
            return False

        # Define a reachable target pose near the current arm position.
        # Using rg6_tcp as the IK link.
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = 0.4
        target.pose.position.y = 0.0
        target.pose.position.z = 0.5
        target.pose.orientation.x = 1.0  # 180° around X (tool pointing down)
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.robot_state.is_diff = True
        req.ik_request.pose_stamped = target
        req.ik_request.ik_link_name = "tool0"
        req.ik_request.timeout.sec = 2

        f = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        if f.result() is None:
            self.get_logger().error("IK service timed out")
            return False
        ec = f.result().error_code.val
        if ec == 1:  # SUCCESS
            sol = f.result().solution.joint_state
            self.get_logger().info(f"  IK solution found ({len(sol.name)} joints)")
            for n, p in zip(sol.name, sol.position):
                if n in UR_JOINTS:
                    self.get_logger().info(f"    {n}: {p:.4f}")
            return True
        else:
            self.get_logger().error(f"  IK failed, error_code={ec}")
            return False

    def test_plan_and_execute(self) -> bool:
        self.get_logger().info("Test 2: /move_action (plan + execute, vel_scale=0.2)")
        if not self.mg.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_action not available")
            return False

        # Conservative target: small joint-space move from current config
        # NOTE: deliberately slow per safe-default policy.
        target_joints = [0.3, -1.4, 1.0, -1.4, 0.0, 0.0]

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnectkConfigDefault"
        req.start_state.is_diff = True

        c = Constraints()
        c.name = "joint_goal"
        for jn, jp in zip(UR_JOINTS, target_joints):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = jp
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal.request = req

        opt = PlanningOptions()
        opt.plan_only = False
        opt.planning_scene_diff.is_diff = True
        opt.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opt

        fut = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("  move_action goal rejected")
            return False
        self.get_logger().info("  goal accepted, awaiting result…")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60.0)
        r = rf.result()
        if r is None:
            self.get_logger().error("  no result"); return False
        ec = r.result.error_code.val
        self.get_logger().info(f"  result error_code={ec} (1 = SUCCESS)")
        return ec == 1


def main():
    rclpy.init()
    n = MoveItTester()
    ok_ik = n.test_ik()
    ok_plan = n.test_plan_and_execute()
    n.destroy_node()
    rclpy.shutdown()
    print("IK:", "PASS" if ok_ik else "FAIL")
    print("PLAN+EXEC:", "PASS" if ok_plan else "FAIL")


if __name__ == "__main__":
    main()

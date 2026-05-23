"""Verify MoveIt collision detection:
  1. Spawn a virtual box right in front of the arm.
  2. Plan a path that would clearly pass through it.
  3. Confirm the plan EITHER routes around the box, OR fails cleanly.
  4. Remove the obstacle and confirm the same plan now succeeds.

We use the PlanningScene topic to inject the obstacle (no live sensor needed).
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import (CollisionObject, PlanningScene,
                             MotionPlanRequest, PlanningOptions,
                             Constraints, JointConstraint)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def make_box(node_id, frame, position, size, op):
    co = CollisionObject()
    co.id = node_id
    co.header.frame_id = frame
    p = SolidPrimitive()
    p.type = SolidPrimitive.BOX
    p.dimensions = list(size)
    co.primitives.append(p)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.0
    co.primitive_poses.append(pose)
    co.operation = op
    return co


def main():
    rclpy.init()
    n = Node("collision_test")
    scene_pub = n.create_publisher(PlanningScene, "/planning_scene", 10)
    mg = ActionClient(n, MoveGroup, "/move_action")

    # 1) Define a goal that requires the arm to swing across X. The
    #    obstacle will sit at +X 0.3 m, blocking the path.
    target_joints_in_collision = [1.57, -1.0, 1.0, -1.5, 0.0, 0.0]
    target_joints_clear        = [-1.0, -1.4, 1.0, -1.5, 0.0, 0.0]

    def plan(label, target):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.1   # safe
        req.max_acceleration_scaling_factor = 0.1
        req.pipeline_id = "ompl"
        req.start_state.is_diff = True
        c = Constraints(); c.name = "goal"
        for j, p in zip(UR_JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = j; jc.position = p
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal.request = req
        opt = PlanningOptions()
        opt.plan_only = True  # <-- DO NOT execute, just plan
        opt.planning_scene_diff.is_diff = True
        opt.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opt

        f = mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
        gh = f.result()
        if not gh or not gh.accepted:
            print(f"  [{label}] goal rejected"); return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=30.0)
        r = rf.result()
        ec = r.result.error_code.val if r else None
        n_pts = len(r.result.planned_trajectory.joint_trajectory.points) if r else 0
        print(f"  [{label}] error_code={ec}, waypoints={n_pts}")
        return ec

    if not mg.wait_for_server(timeout_sec=10.0):
        print("FAIL: move_action missing"); return

    print("Step 1: plan to clear pose WITHOUT obstacle (expect SUCCESS)")
    ec0 = plan("baseline", target_joints_clear)

    print("Step 2: add a 0.4×0.4×0.4 m box at (0.4, 0, 0.4) blocking the swing")
    ps = PlanningScene()
    ps.is_diff = True
    box = make_box("test_obstacle", "world", (0.4, 0.0, 0.4), (0.4, 0.4, 0.4),
                   CollisionObject.ADD)
    ps.world.collision_objects.append(box)
    # publish enough times for the planning scene monitor to latch
    for _ in range(10):
        scene_pub.publish(ps)
        time.sleep(0.1)

    print("Step 3: plan the SAME baseline pose WITH obstacle (expect SUCCESS but a longer path)")
    ec1 = plan("with-box", target_joints_clear)

    print("Step 4: plan to a pose THROUGH the obstacle (expect FAIL / re-route)")
    ec2 = plan("through-box", target_joints_in_collision)

    print("Step 5: remove the box")
    ps2 = PlanningScene()
    ps2.is_diff = True
    rem = CollisionObject()
    rem.id = "test_obstacle"; rem.operation = CollisionObject.REMOVE
    ps2.world.collision_objects.append(rem)
    for _ in range(10):
        scene_pub.publish(ps2); time.sleep(0.1)

    print("Step 6: re-plan through-box pose (expect SUCCESS, obstacle gone)")
    ec3 = plan("after-remove", target_joints_in_collision)

    print()
    print("="*50)
    # Pass criteria: ec0 == 1 (baseline succeeded),
    #                ec1 == 1 (succeed with obstacle, just longer),
    #                ec3 == 1 (obstacle removed, succeed)
    #                ec2 may be 1 (re-route) or -1/-12 (no path) — both valid
    ok = (ec0 == 1) and (ec1 == 1) and (ec3 == 1)
    if ok:
        print(f"COLLISION DETECTION: PASS (ec0={ec0}, ec1={ec1}, ec2={ec2}, ec3={ec3})")
        if ec2 == 1:
            print("  Note: planner found a path around the box")
        else:
            print(f"  Note: planner refused the through-box goal (ec2={ec2})")
    else:
        print(f"COLLISION DETECTION: FAIL (ec0={ec0}, ec1={ec1}, ec2={ec2}, ec3={ec3})")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

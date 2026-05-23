"""Slow combined demo: UR arm wave + gripper open/close, kept at conservative
speeds throughout per the safe-default policy.

  Arm: 4 waypoints over 16 s  (≥ 5 s between holds)
  Gripper: 4-step open/close cycle, 3 s per hold
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# Safe defaults
INTER_WP = 5.0       # seconds between arm waypoints
GRIPPER_HOLD = 3.0   # seconds per gripper transition


def make_traj(joint_names, waypoints):
    t = JointTrajectory()
    t.joint_names = joint_names
    for pos, sec in waypoints:
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.velocities = [0.0] * len(pos)
        whole = int(sec)
        frac = int((sec - whole) * 1e9)
        pt.time_from_start = Duration(sec=whole, nanosec=frac)
        t.points.append(pt)
    return t


def run_action(node, client, traj, label, timeout):
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"[{label}] action server missing"); return False
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    f = client.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    gh = f.result()
    if not gh or not gh.accepted:
        node.get_logger().error(f"[{label}] rejected"); return False
    node.get_logger().info(f"[{label}] accepted, executing…")
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=timeout)
    r = rf.result()
    code = r.result.error_code if r else None
    node.get_logger().info(f"[{label}] done error_code={code}")
    return code == 0


def hold_gripper(node, pub, value, seconds, label):
    traj = JointTrajectory()
    traj.joint_names = ["rg6_joint"]
    pt = JointTrajectoryPoint()
    pt.positions = [float(value)]
    pt.velocities = [0.0]
    sec = int(seconds)
    nsec = int((seconds - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=nsec)
    traj.points.append(pt)
    node.get_logger().info(f"[gripper] → {label} ({value:.2f} rad) over {seconds:.1f}s")
    pub.publish(traj)
    deadline = time.time() + seconds + 0.3
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    n = Node("demo_full_safe")

    arm = ActionClient(n, FollowJointTrajectory,
                       "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    grip_pub = n.create_publisher(JointTrajectory,
                                  "/rg6_gripper_controller/joint_trajectory", 10)
    time.sleep(0.5)

    # 1. ARM — slow 4-waypoint wave
    arm_traj = make_traj(UR_JOINTS, [
        ([0.0,   -1.5708, 0.0,    -1.5708, 0.0, 0.0],          INTER_WP * 1),
        ([0.7,   -1.2,    1.0,    -1.4,    0.4, 0.0],          INTER_WP * 2),
        ([-0.7,  -1.2,    1.0,    -1.4,   -0.4, 0.0],          INTER_WP * 3),
        ([0.0,   -1.5708, 0.0,    -1.5708, 0.0, 0.0],          INTER_WP * 4),
    ])
    run_action(n, arm, arm_traj, "arm wave", timeout=INTER_WP * 4 + 5)

    time.sleep(1.0)

    # 2. GRIPPER — slow open / close cycle
    hold_gripper(n, grip_pub, 1.30, GRIPPER_HOLD, "full open")
    hold_gripper(n, grip_pub, 0.65, GRIPPER_HOLD, "half")
    hold_gripper(n, grip_pub, 0.00, GRIPPER_HOLD, "closed")
    hold_gripper(n, grip_pub, 0.08, GRIPPER_HOLD, "safe boot")

    n.get_logger().info("demo complete")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

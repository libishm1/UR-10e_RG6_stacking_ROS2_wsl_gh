"""Drive the arm + open/close the gripper so the user can see both moving."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def make_traj(joint_names, waypoints):
    """waypoints: list of (positions, seconds)"""
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


def run(client, traj, name):
    if not client.wait_for_server(timeout_sec=5.0):
        print(f"[{name}] action server not available"); return False
    g = FollowJointTrajectory.Goal(); g.trajectory = traj
    f = client.send_goal_async(g)
    rclpy.spin_until_future_complete(client._node, f, timeout_sec=5.0)
    gh = f.result()
    if not gh or not gh.accepted:
        print(f"[{name}] rejected"); return False
    print(f"[{name}] accepted")
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(client._node, rf, timeout_sec=20.0)
    r = rf.result()
    print(f"[{name}] done, error_code={r.result.error_code if r else 'NONE'}")
    return r is not None and r.result.error_code == 0


def main():
    rclpy.init()
    node = Node("arm_gripper_demo")
    arm = ActionClient(node, FollowJointTrajectory,
                       "/scaled_joint_trajectory_controller/follow_joint_trajectory")
    arm._node = node
    grip = ActionClient(node, FollowJointTrajectory,
                        "/joint_trajectory_controller/follow_joint_trajectory")
    grip._node = node

    # Arm trajectory — a clear, slow wave
    arm_traj = make_traj(UR_JOINTS, [
        ([0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0], 3.0),
        ([1.0,  -1.0,    1.2,  -1.5,    0.5, 0.0], 6.0),
        ([-1.0, -1.0,    1.2,  -1.5,   -0.5, 0.0], 9.0),
        ([0.0,  -1.5708, 0.0,  -1.5708, 0.0, 0.0], 12.0),
    ])

    print("→ ARM: 4-waypoint wave (12 s)")
    if not run(arm, arm_traj, "arm"):
        return

    # Gripper open/close — drives rg6_left_finger_joint; the right finger mimics
    # joint_trajectory_controller is inactive by default; activate it first.
    import subprocess
    subprocess.run(["bash", "-lc",
                    "source /opt/ros/humble/setup.bash && source ~/ur_rg6_ws/install/setup.bash && "
                    "ros2 control switch_controllers --activate joint_trajectory_controller 2>&1 | head -3"])
    time.sleep(1.0)
    grip_traj = make_traj(["rg6_left_finger_joint"], [
        ([0.040], 2.0),   # open
        ([0.0],   4.0),   # close
        ([0.040], 6.0),   # open
        ([0.020], 8.0),   # half
    ])
    print("→ GRIPPER: open/close/open/half (8 s)")
    run(grip, grip_traj, "gripper")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

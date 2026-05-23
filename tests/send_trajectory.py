"""Send a multi-pose joint trajectory and verify TF transforms after."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Each pose is (joint positions, seconds_from_start)
POSES = [
    ([0.0,   -1.5708, 0.0,     -1.5708, 0.0, 0.0], 3.0),   # home-ish
    ([0.8,   -1.2,    1.0,     -1.3,    0.5, 0.5], 6.0),   # away+up
    ([-0.8,  -1.2,    1.0,     -1.3,   -0.5, 0.0], 9.0),   # mirror
    ([0.0,   -1.5708, 0.0,     -1.5708, 0.0, 0.0], 12.0),  # back home
]


class Sender(Node):
    def __init__(self):
        super().__init__("trajectory_sender")
        self.client = ActionClient(
            self, FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory")

    def send(self):
        self.get_logger().info("Waiting for action server…")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("scaled_joint_trajectory_controller action not available")
            return False

        traj = JointTrajectory()
        traj.joint_names = UR_JOINTS
        for positions, t in POSES:
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.0] * 6
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f"Sending {len(POSES)} waypoints, last @ t={POSES[-1][1]}s")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected")
            return False

        self.get_logger().info("Accepted; waiting for completion…")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=15.0)
        result = rf.result()
        if result is None:
            self.get_logger().error("No result (timeout)")
            return False
        self.get_logger().info(f"Done. error_code={result.result.error_code}")
        return result.result.error_code == 0


def main():
    rclpy.init()
    n = Sender()
    ok = n.send()
    n.destroy_node()
    rclpy.shutdown()
    print("OK" if ok else "FAIL")


if __name__ == "__main__":
    main()

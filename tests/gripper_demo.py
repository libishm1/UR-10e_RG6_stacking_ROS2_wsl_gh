"""Drive ONLY the RG6 gripper through a visible open/close cycle.

Holds each commanded position long enough for the position controller to
settle (the trick with JointGroupPositionController is that a single
publish doesn't latch — you have to hold the topic until the position
matches the command).
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


SEQUENCE = [
    ("full open  (1.30 rad)", 1.30, 2.5),
    ("half open  (0.65 rad)", 0.65, 2.5),
    ("closed     (0.00 rad)", 0.00, 2.5),
    ("half open  (0.65 rad)", 0.65, 2.5),
    ("safe boot  (0.08 rad)", 0.08, 2.0),
]


class GripperDemo(Node):
    def __init__(self):
        super().__init__("rg6_gripper_demo")
        self.pub = self.create_publisher(
            JointTrajectory, "/rg6_gripper_controller/joint_trajectory", 10)

    def hold(self, value: float, seconds: float, label: str):
        traj = JointTrajectory()
        traj.joint_names = ["rg6_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(value)]
        pt.velocities = [0.0]
        sec = int(seconds)
        nsec = int((seconds - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.get_logger().info(f"→ {label}")
        self.pub.publish(traj)
        # Let the trajectory play out
        time.sleep(seconds + 0.3)


def main():
    rclpy.init()
    n = GripperDemo()
    # Give pub time to attach
    time.sleep(0.5)
    for label, val, dur in SEQUENCE:
        n.hold(val, dur, label)
    n.get_logger().info("DONE")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

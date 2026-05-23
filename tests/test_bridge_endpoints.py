"""Verify every endpoint the Grasshopper bridge needs is healthy on the
live ROS 2 graph. This is the headless equivalent of what roslibpy would
exercise through rosbridge — same topics, same services, same actions.

Run with the control + move_group stack already up.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


REQUIRED_UR_JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
}
REQUIRED_GRIPPER_JOINTS = {"rg6_joint"}

REQUIRED_TOPICS = [
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/tf", "tf2_msgs/msg/TFMessage"),
    ("/tf_static", "tf2_msgs/msg/TFMessage"),
    ("/rg6_gripper_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory"),
]

REQUIRED_ACTIONS = [
    "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    "/rg6_gripper_controller/follow_joint_trajectory",
    "/move_action",
]


class BridgeProbe(Node):
    def __init__(self):
        super().__init__("bridge_endpoint_probe")
        self.joint_msg = None
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, m):
        self.joint_msg = m


def main():
    rclpy.init()
    n = BridgeProbe()
    failures = []

    # 1. /joint_states content
    print("Test 1 — /joint_states content")
    deadline = time.time() + 5
    while n.joint_msg is None and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.joint_msg is None:
        print("  FAIL: no message in 5 s")
        failures.append("joint_states")
    else:
        got = set(n.joint_msg.name)
        missing_ur = REQUIRED_UR_JOINTS - got
        missing_grip = REQUIRED_GRIPPER_JOINTS - got
        if missing_ur or missing_grip:
            print(f"  FAIL: missing joints UR={missing_ur} GRIP={missing_grip}")
            failures.append("joint_states_joints")
        else:
            print(f"  PASS: {len(got)} joints, all required present")

    # 2. Topic types
    print("Test 2 — published topic types")
    topic_types = dict(n.get_topic_names_and_types())
    for topic, expected_type in REQUIRED_TOPICS:
        actual = topic_types.get(topic, [])
        if expected_type in actual:
            print(f"  PASS: {topic} ({expected_type})")
        else:
            print(f"  FAIL: {topic} expected {expected_type}, got {actual}")
            failures.append(f"topic:{topic}")

    # 3. Actions visible
    print("Test 3 — action servers")
    # Use the underlying graph query
    from rclpy.action import get_action_names_and_types
    actions = dict(get_action_names_and_types(n))
    for action in REQUIRED_ACTIONS:
        if action in actions:
            print(f"  PASS: {action}")
        else:
            print(f"  FAIL: {action} not advertised. Available: {list(actions.keys())[:5]}...")
            failures.append(f"action:{action}")

    # 4. /compute_ik service
    print("Test 4 — /compute_ik service")
    services = dict(n.get_service_names_and_types())
    if "/compute_ik" in services:
        print("  PASS: /compute_ik present")
    else:
        print("  FAIL: /compute_ik missing")
        failures.append("compute_ik")

    # 5. Send a gripper command (safe, low) and verify state moves
    print("Test 5 — gripper command flow (safe, 0.3 rad, JointTrajectory)")
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    pub = n.create_publisher(JointTrajectory,
                             "/rg6_gripper_controller/joint_trajectory", 10)
    initial = n.joint_msg.position[list(n.joint_msg.name).index("rg6_joint")]
    print(f"  initial rg6_joint: {initial:.3f}")
    traj = JointTrajectory()
    traj.joint_names = ["rg6_joint"]
    pt = JointTrajectoryPoint()
    pt.positions = [0.3]; pt.velocities = [0.0]
    pt.time_from_start = Duration(sec=2, nanosec=0)
    traj.points.append(pt)
    # Wait for the publisher to attach to the topic, then publish
    time.sleep(0.5)
    pub.publish(traj)
    deadline = time.time() + 3
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    final = n.joint_msg.position[list(n.joint_msg.name).index("rg6_joint")]
    print(f"  final rg6_joint: {final:.3f}")
    if abs(final - 0.3) < 0.05:
        print("  PASS: gripper followed trajectory")
    else:
        print(f"  FAIL: expected ~0.3, got {final:.3f}")
        failures.append("gripper_flow")

    n.destroy_node()
    rclpy.shutdown()

    print()
    print("=" * 50)
    if failures:
        print(f"FAILURES: {failures}")
    else:
        print("ALL BRIDGE ENDPOINTS HEALTHY")
    print("=" * 50)


if __name__ == "__main__":
    main()

"""Direct collision-detection check via /check_state_validity.

  1. Validate the CURRENT joint state — expect valid=true.
  2. Inject a box that surrounds the gripper.
  3. Validate the CURRENT joint state again — expect valid=false (in collision
     with our box).
  4. Remove the box, validate once more — expect valid=true.

This isolates collision detection from path planning.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import PlanningScene, CollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class CollisionTester(Node):
    def __init__(self):
        super().__init__("collision_v2_tester")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)
        self.cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

    def check(self) -> bool:
        if self.js is None:
            return None
        req = GetStateValidity.Request()
        req.group_name = "ur_manipulator"
        rs = RobotState()
        rs.joint_state = self.js
        req.robot_state = rs
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        r = f.result()
        if r is None:
            return None
        return r.valid

    def push_box(self, frame, pos, size, add=True):
        ps = PlanningScene(); ps.is_diff = True
        co = CollisionObject()
        co.id = "test_box"
        co.header.frame_id = frame
        if add:
            p = SolidPrimitive(); p.type = SolidPrimitive.BOX
            p.dimensions = list(size)
            co.primitives.append(p)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
        for _ in range(10):
            self.scene_pub.publish(ps)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    n = CollisionTester()
    # Wait for first joint_state + service
    deadline = time.time() + 5
    while (n.js is None) and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.cli.wait_for_service(timeout_sec=5.0):
        print("FAIL: /check_state_validity missing"); return

    print("Step 1: validate current state (no obstacle) — expect valid=True")
    v0 = n.check()
    print(f"  valid = {v0}")

    print("Step 2: inject a 1×1×1 m box centred on the gripper at world(0,0.44,0.7)")
    n.push_box("world", (0.0, 0.44, 0.7), (1.0, 1.0, 1.0), add=True)
    time.sleep(1.0)

    print("Step 3: validate current state (gripper inside box) — expect valid=False")
    v1 = n.check()
    print(f"  valid = {v1}")

    print("Step 4: remove the box")
    n.push_box("world", (0.0, 0.44, 0.7), (1.0, 1.0, 1.0), add=False)
    time.sleep(1.0)

    print("Step 5: validate current state (clear again) — expect valid=True")
    v2 = n.check()
    print(f"  valid = {v2}")

    n.destroy_node()
    rclpy.shutdown()

    print()
    print("=" * 50)
    if v0 is True and v1 is False and v2 is True:
        print("COLLISION DETECTION: PASS")
    else:
        print(f"COLLISION DETECTION: FAIL  (v0={v0}, v1={v1}, v2={v2})")


if __name__ == "__main__":
    main()

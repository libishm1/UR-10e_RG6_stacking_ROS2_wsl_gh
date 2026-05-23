"""Benchmark IK solver — call /compute_ik N times and report median latency."""
import rclpy
import time
import statistics
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped


N = 50


def main():
    rclpy.init()
    n = Node("ik_bench")
    cli = n.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("FAIL: compute_ik missing"); return

    req = GetPositionIK.Request()
    req.ik_request.group_name = "ur_manipulator"
    req.ik_request.robot_state.is_diff = True
    req.ik_request.ik_link_name = "tool0"
    req.ik_request.timeout.sec = 0
    req.ik_request.timeout.nanosec = 100_000_000  # 100 ms cap per call

    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.orientation.x = 1.0  # tool down
    target.pose.position.x = 0.4
    target.pose.position.z = 0.45

    latencies = []
    successes = 0
    for i in range(N):
        # small jitter so we don't hit the same pose every iteration
        target.pose.position.y = (i % 10 - 5) * 0.02
        req.ik_request.pose_stamped = target
        t0 = time.perf_counter()
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
        dt = (time.perf_counter() - t0) * 1000.0  # ms
        if f.result() and f.result().error_code.val == 1:
            successes += 1
            latencies.append(dt)

    n.destroy_node()
    rclpy.shutdown()

    print(f"IK benchmark: {N} calls, {successes} solved")
    if latencies:
        print(f"  median: {statistics.median(latencies):.2f} ms")
        print(f"  mean:   {statistics.mean(latencies):.2f} ms")
        print(f"  p95:    {sorted(latencies)[int(0.95*len(latencies))-1]:.2f} ms")
        print(f"  min:    {min(latencies):.2f} ms")
        print(f"  max:    {max(latencies):.2f} ms")


if __name__ == "__main__":
    main()

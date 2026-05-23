"""Calibrate the rg6_joint (rad) → finger width (mm) mapping using live TF.

Sweeps rg6_joint from 0 to 1.3 rad, queries the world distance between
rg6_finger_1_flex_finger and rg6_finger_2_flex_finger, records (rad → width).
Then fits a polynomial and writes the lookup + the inverse mapping to a YAML
the play scripts can import.
"""
import time
import math
import yaml
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration as Dur


SAMPLES = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
           0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95,
           1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30]

GRIPPER_TOPIC = "/rg6_gripper_controller/joint_trajectory"


class Calibrator(Node):
    def __init__(self):
        super().__init__("rg6_width_calibrator")
        self.pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def set_joint(self, angle, hold_sec=1.2):
        msg = JointTrajectory()
        msg.joint_names = ["rg6_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(angle)]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(pt)
        self.pub.publish(msg)
        # Let the controller execute + TF settle
        deadline = time.time() + hold_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def width_at(self, angle):
        self.set_joint(angle)
        # Read both flex_finger transforms in world frame
        try:
            t1 = self.buf.lookup_transform(
                "world", "rg6_finger_1_flex_finger",
                rclpy.time.Time(), Dur(seconds=1.0))
            t2 = self.buf.lookup_transform(
                "world", "rg6_finger_2_flex_finger",
                rclpy.time.Time(), Dur(seconds=1.0))
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed at angle {angle}: {e}")
            return None
        dx = t1.transform.translation.x - t2.transform.translation.x
        dy = t1.transform.translation.y - t2.transform.translation.y
        dz = t1.transform.translation.z - t2.transform.translation.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main():
    rclpy.init()
    n = Calibrator()
    # Wait a moment for TF + publisher to wire up
    time.sleep(1.5)

    data = []
    for ang in SAMPLES:
        w = n.width_at(ang)
        if w is None:
            continue
        w_mm = w * 1000.0
        print(f"  rg6_joint = {ang:.3f} rad   →   width = {w_mm:7.2f} mm")
        data.append((ang, w_mm))

    # Fit a 3rd-degree polynomial in BOTH directions
    import numpy as np
    A = np.array([a for a, _ in data])
    W = np.array([w for _, w in data])
    coeff_w_of_a = np.polyfit(A, W, 3).tolist()   # width(angle)
    coeff_a_of_w = np.polyfit(W, A, 3).tolist()   # angle(width)

    print()
    print("Polynomial fits (cubic):")
    print(f"  width_mm(angle_rad)  = {coeff_w_of_a}")
    print(f"  angle_rad(width_mm)  = {coeff_a_of_w}")

    # Also report the simple linear approximation we were using for comparison
    print()
    LINEAR = 1.3 / 160.0  # rad per mm
    err_max = 0.0
    print(f"{'rad':>7}  {'real mm':>9}  {'linear mm':>10}  {'err mm':>7}")
    for a, w in data:
        approx = a / LINEAR
        err = abs(approx - w)
        err_max = max(err_max, err)
        print(f"  {a:5.2f}    {w:7.2f}   {approx:8.2f}    {err:5.2f}")
    print(f"  max linear-vs-real error: {err_max:.2f} mm")

    # Save the calibration
    out = {
        "rg6_joint_to_width_mm": {
            "samples": [{"angle_rad": float(a), "width_mm": float(w)} for a, w in data],
            "cubic_width_of_angle": coeff_w_of_a,
            "cubic_angle_of_width": coeff_a_of_w,
            "min_angle_rad": float(min(A)),
            "max_angle_rad": float(max(A)),
            "min_width_mm": float(min(W)),
            "max_width_mm": float(max(W)),
        }
    }
    out_path = "/home/libi/ur_rg6_ws/src/ur10e_rg6_moveit_config/config/rg6_width_calibration.yaml"
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"\nSaved → {out_path}")

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

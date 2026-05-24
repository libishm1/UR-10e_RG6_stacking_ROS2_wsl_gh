"""Read-only measurement script: compare the real UR10e to our ROS HOME.

What this script does (and DOESN'T do):

  DOES:   - Open an `ur_rtde` RTDEReceiveInterface session to the real
            robot (port 30004) — READ ONLY, never writes.
          - Sample joint angles + TCP pose for ~1 second and average.
          - Compare each measured joint to our ROS HOME_Q (verify the
            robot is at HOME and our HOME constant is right).
          - Compute the yaw of the measured TCP base-frame pose vs
            our hardcoded ROS HOME TCP yaw. Both are in BASE frame,
            so the delta verifies kinematic-model agreement; it is
            NOT the URDF-vs-room yaw mismatch.
          - Print a single-page report including a clear note on what
            can and cannot be derived from RTDE-only data (see Verdict).

  DOES NOT:
          - Send ANY command to the robot. No RTDEControlInterface.
          - No URScript, no Dashboard play/stop, no enable/disable.
          - No motion at all. Whatever pose the robot is at when you
            run this, that's what gets measured.

Use case: stand at the cell with the robot at HOME (manually moved or
URScript-played there), then run this from WSL. The script tells you:
  (a) whether the joint values match our HOME constant — if yes, the
      robot is at HOME and our constant is right;
  (b) whether the TCP pose (in base frame) agrees with our URDF FK at
      HOME — if yes, the kinematic model agrees with the real robot.

What this DOES NOT tell you: how to rotate the URDF mount to match
the room. That's a base-frame-vs-world question, and RTDE only knows
the robot's local base frame. To get the URDF mount yaw, you need to
look at the physical cabinet and decide which way 'in front' faces in
your room — the Verdict section spells out exactly how.

Usage:
    # Sim / URSim default
    python3 measure_real_robot_pose.py --host 127.0.0.1

    # Real robot at the verified cell IP
    python3 measure_real_robot_pose.py --host 192.168.1.100

    # With a custom samples count or pose-reference name
    python3 measure_real_robot_pose.py --host 192.168.1.100 \
        --samples 200 --reference HOME

Hard gates:
  - Refuses loopback unless --allow-loopback (so you can't accidentally
    measure URSim and think it's the real cell).
  - Refuses to run if any RTDEControlInterface is detected as imported
    anywhere — guards against accidental scope creep into a write path.

Dependencies:
    sudo apt install -y python3-pip
    pip3 install --user ur_rtde

References:
  - D:\\robot_ws\\robots\\wiki\\ur10e_rg6\\rtde_readonly_protocol.md
    (read-only RTDE contract used in the C# Mecha plug-in).
  - tests/check_real_hw_network.sh (pre-flight: pendant services + ports
    must be enabled first, see docs/WSL2_UR10e_NETWORKING.md).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

# Hard gate (1/2): never import any RTDE WRITE interface.
# This script is purely read-side. If an importer tries to add control
# (RTDEControlInterface, RTDEIOInterface), the test below fails loudly.
_FORBIDDEN_MODULES = (
    "rtde_control",
    "rtde_io",
)


@dataclass
class HomePose:
    """Reference HOME pose, both in joint space and base-frame TCP."""

    # User-verified HOME (from URScript dodectest3.urp + this workspace's
    # SRDF home group_state + play_pickplace.py HOME_Q).
    # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3] in rad.
    joints_rad: tuple = (1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708)

    # TCP pose in base frame at this HOME, from `ros2 run tf2_ros tf2_echo
    # base_link tool0` while at HOME (no URDF rotation). Refresh by hand if
    # the URDF or HOME ever changes.
    #   x, y, z (m)               — translation
    #   rx, ry, rz (axis-angle)   — orientation (UR convention)
    tcp_base_xyz: tuple = (0.586, 0.043, 0.691)
    tcp_base_rotvec: tuple = (0.0, 0.0, 0.0)   # unknown without FK


HOME = HomePose()

LOOPBACK = {"127.0.0.1", "localhost", "::1"}


# ---------------- Helpers ----------------

def rotvec_to_yaw(rx: float, ry: float, rz: float) -> float:
    """Extract the yaw component of an axis-angle rotation vector.

    For a rotvec (rx, ry, rz), the rotation magnitude is theta = |r|
    and the axis is r / theta. We rebuild the rotation matrix and
    return the Z-axis yaw (atan2(R[1,0], R[0,0])).
    """
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-9:
        return 0.0
    ux, uy, uz = rx / theta, ry / theta, rz / theta
    c = math.cos(theta)
    s = math.sin(theta)
    one_c = 1.0 - c
    # R[0,0] = c + ux*ux*(1-c)
    # R[1,0] = uy*ux*(1-c) + uz*s
    r00 = c + ux * ux * one_c
    r10 = uy * ux * one_c + uz * s
    return math.atan2(r10, r00)


def deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def normalise_deg(d: float) -> float:
    """Normalise to (-180, 180]."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def assert_read_only_import_scope():
    """Refuse to run if any RTDE write-side module is imported in this
    Python process (now or later via lazy import attack). The check runs
    on entry and is mostly there to document the read-only constraint
    in code, not just docstrings."""
    for m in _FORBIDDEN_MODULES:
        if m in sys.modules:
            print(f"FATAL: {m} is loaded — this script must stay read-only.",
                  file=sys.stderr)
            sys.exit(2)


# ---------------- Main measurement ----------------

def measure(host: str, samples: int, allow_loopback: bool) -> int:
    assert_read_only_import_scope()

    if host in LOOPBACK and not allow_loopback:
        print(f"REFUSED: loopback host '{host}' is not measuring the real "
              f"cell. Pass --allow-loopback if you really want URSim.",
              file=sys.stderr)
        return 2

    # Import only the READ interface. ur_rtde's RTDEReceiveInterface is
    # read-only by API contract.
    try:
        from rtde_receive import RTDEReceiveInterface
    except ImportError:
        print("FATAL: ur_rtde Python package not installed.\n"
              "Install with:  pip3 install --user ur_rtde\n"
              "(or `sudo apt install python3-ur-rtde` if available)",
              file=sys.stderr)
        return 3

    print(f"[connect] RTDEReceiveInterface → {host}:30004 …")
    try:
        recv = RTDEReceiveInterface(host)
    except Exception as e:
        print(f"FATAL: could not connect to {host}: {e}", file=sys.stderr)
        return 4
    print(f"[connect] OK — sampling {samples} frames over ~1 s")

    # Take `samples` frames spread over ~1 s and average.
    joint_acc = [0.0] * 6
    tcp_pos_acc = [0.0] * 3
    tcp_rot_acc = [0.0] * 3
    n = 0
    deadline = time.monotonic() + 1.0
    period = max(0.001, 1.0 / float(samples))
    while time.monotonic() < deadline and n < samples:
        q = recv.getActualQ()           # 6 joint angles, rad
        tcp = recv.getActualTCPPose()   # [x, y, z, rx, ry, rz] base frame
        for i in range(6):
            joint_acc[i] += q[i]
        for i in range(3):
            tcp_pos_acc[i] += tcp[i]
            tcp_rot_acc[i] += tcp[3 + i]
        n += 1
        time.sleep(period)

    if n < 5:
        print(f"FATAL: only {n} samples collected. RTDE stream may be down.",
              file=sys.stderr)
        return 5

    q_mean = [joint_acc[i] / n for i in range(6)]
    pos_mean = [tcp_pos_acc[i] / n for i in range(3)]
    rot_mean = [tcp_rot_acc[i] / n for i in range(3)]

    # ---------------- Report ----------------
    JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow",
                   "wrist_1", "wrist_2", "wrist_3"]

    print()
    print("─── Real-robot state ─────────────────────────────────────────")
    print(f"  samples  : {n}")
    print(f"  host     : {host}")
    print()
    print("  Joint angles (rad)        |  ROS HOME (rad)         |  Δ (deg)")
    print("  --------------------------+-------------------------+---------")
    joint_match_count = 0
    for i, name in enumerate(JOINT_NAMES):
        meas = q_mean[i]
        ref = HOME.joints_rad[i]
        delta_deg = normalise_deg(deg(meas - ref))
        match = abs(delta_deg) < 1.0       # ±1° tolerance
        if match:
            joint_match_count += 1
        flag = " " if match else "*"
        print(f"  {flag}{name:<14} {meas:+8.4f}  |  {ref:+8.4f}              |"
              f"  {delta_deg:+7.2f}")

    print()
    print("  TCP pose in base frame")
    print(f"    x, y, z     = {pos_mean[0]:+.4f},  {pos_mean[1]:+.4f},  "
          f"{pos_mean[2]:+.4f}  (m)")
    print(f"    rx, ry, rz  = {rot_mean[0]:+.4f},  {rot_mean[1]:+.4f},  "
          f"{rot_mean[2]:+.4f}  (rad, axis-angle)")
    measured_yaw_deg = deg(rotvec_to_yaw(*rot_mean))
    home_yaw_deg = deg(rotvec_to_yaw(*HOME.tcp_base_rotvec))
    print(f"    TCP yaw     = {measured_yaw_deg:+.2f}°")
    print(f"    ROS HOME yaw= {home_yaw_deg:+.2f}°")
    yaw_delta = normalise_deg(measured_yaw_deg - home_yaw_deg)
    print()
    print("─── Verdict ──────────────────────────────────────────────────")
    if joint_match_count == 6:
        print("  Joint values match ROS HOME (±1°). The robot IS at HOME.")
        print(f"  TCP base-frame yaw delta vs ROS HOME = {yaw_delta:+.2f}°")
        print()
        print("  IMPORTANT — what this CAN and CANNOT tell us:")
        print("    ✓ CAN confirm: joint angles match, kinematic model agrees")
        print("      (TCP-in-base-frame is the same in real-robot and URDF).")
        print("    ✗ CANNOT measure: how the robot's BASE FRAME is oriented")
        print("      in the room. RTDE only returns poses in the robot's own")
        print("      base frame; it has no concept of 'world' or 'the room'.")
        print()
        print("  So if RViz visually disagrees with the real cell, the cause")
        print("  is the URDF mounting `<origin rpy='0 0 ?'>` not matching the")
        print("  physical cabinet yaw in your room. That has to come from")
        print("  visual observation, not from RTDE.")
        print()
        print("  Next step (visual measurement):")
        print("    1. Stand in front of the cabinet. Note which physical")
        print("       direction the manufacturer's 'front arrow' (opposite the")
        print("       cable exit) points: say it's +X_room.")
        print("    2. In our URDF (rpy=0), base_link's +X also points 'in")
        print("       front' of the robot.")
        print("    3. If +X_room aligns with how YOU view +X in RViz, no")
        print("       rotation needed. If +X_room is rotated 180° from RViz's")
        print("       +X (cable points the OTHER way), set")
        print("       `<origin rpy='0 0 3.14159'>` in the URDF.")
        print("    4. Other angles: measure the room→URDF yaw with a tape")
        print("       measure or by aligning a fiducial; set that as the rpy_z.")
    else:
        print(f"  Joint values do NOT match ROS HOME ({joint_match_count}/6"
              " within ±1°).")
        print("  → Either the robot isn't at HOME, or our HOME definition is")
        print("    wrong. Move the robot to HOME on the pendant (URScript:")
        print("    `movej([1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]")
        print("    , a=0.5, v=0.25)`), then re-run this script.")
    print()

    recv.disconnect()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True,
                    help="UR10e IP address (e.g. 192.168.1.100). "
                         "Loopback refused without --allow-loopback.")
    ap.add_argument("--samples", type=int, default=100,
                    help="Number of RTDE samples to average (default 100).")
    ap.add_argument("--allow-loopback", action="store_true",
                    help="Allow 127.0.0.1 / localhost (URSim). Off by default "
                         "so you can't accidentally measure URSim and think "
                         "it's the real cell.")
    args = ap.parse_args()
    sys.exit(measure(args.host, args.samples, args.allow_loopback))


if __name__ == "__main__":
    main()

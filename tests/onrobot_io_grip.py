#!/usr/bin/env python3
"""OnRobot RG6 binary grip control via ur_robot_driver's set_io service.

Bypasses the URCap entirely: tool digital output 0 (PIN_TOOL_DOUT0 = 16) is
wired through the OnRobot Quick Changer to the gripper's internal MCU,
which reads the level (HIGH = close, LOW = open) and actuates the
fingers. The OnRobot URCap on the pendant is GUI-only and is NOT
required for this path; in fact it must be SUPPRESSED via
Installation → General → Tool I/O → "Controlled by: User" so the URCap
doesn't fight our set_io writes.

This is BINARY close/open ONLY. Continuous width / variable force require
either the OnRobot Compute Box (Modbus TCP) or the OnRobot RS-485 URCap
(direct serial Modbus from the ROS host). For the wood-block stacking
demo, binary is sufficient — every pick is the same physical block.

Why we don't use rg_grip() via /urscript_interface/script_command:
  The OnRobot URCap's rg_grip is a Java-backed PolyScope program-tree
  node, not a URScript-text function. Rebuilding external_control.urp
  with an OnRobot RG node above External Control loads the URCap GUI
  preamble but does NOT inject rg_grip into the URScript runtime
  namespace that External Control's socket evaluates. Empirically
  verified 2026-05-26 (URP rebuild test); see SESSION_HANDOFF.md.

Grip-detect feedback (TODO):
  The OnRobot RG6 reports grip-detect via tool digital input 0. In our
  Humble ur_msgs version, neither IOStates nor ToolDataMsg has an
  obvious tool_digital_input_states field. The tool DI may appear in
  IOStates.digital_in_states at pin 16/17 — runtime-verify when the
  driver is connected to real hardware:

      ros2 topic echo /io_and_status_controller/io_states --once | head -40

  For now, this helper uses a fixed timeout (1.5 s) which is empirically
  generous for the RG6 in its 0-160 mm full-stroke speed. Replace with
  edge-detection on the right Digital[] entry once verified.
"""
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO


# Constants from ur_msgs/srv/SetIO (verified via ros2 interface show 2026-05-27)
FUN_SET_DIGITAL_OUT = 1
PIN_TOOL_DOUT0 = 16          # OnRobot RG6 command line (HIGH=close, LOW=open)
PIN_TOOL_DOUT1 = 17          # unused for RG6; some OnRobot grippers use it for force toggle
STATE_ON = 1.0
STATE_OFF = 0.0

# Timing budget. RG6 full-stroke (0 ↔ 160 mm) takes ~1 s at default speed.
# Add safety margin for under-load close.
GRIP_DETECT_TIMEOUT_S = 1.5
SETIO_CALL_TIMEOUT_S = 2.0


class OnRobotToolIOGrip:
    """Wrap the SetIO service for binary OnRobot RG6 control.

    Usage:
        grip = OnRobotToolIOGrip(node)
        ok = grip.connect()              # waits for service
        grip.close_blocking()            # close + fixed timeout sleep
        grip.open()                      # open (non-blocking)
    """

    def __init__(self, node: Node, service_name: str = "/io_and_status_controller/set_io"):
        self._node = node
        self._service_name = service_name
        self._cli = node.create_client(SetIO, service_name)

    def connect(self, timeout_s: float = 5.0) -> bool:
        """Wait for the SetIO service to be available. Returns True on success."""
        if not self._cli.wait_for_service(timeout_sec=timeout_s):
            self._node.get_logger().error(
                f"OnRobotToolIOGrip: service '{self._service_name}' not available within {timeout_s}s. "
                "Is ur_ros2_control_node running with real or fake hardware?")
            return False
        return True

    def _call(self, pin: int, state: float) -> bool:
        """Issue one SetIO call. Returns True only on response.success."""
        req = SetIO.Request()
        req.fun = FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = state
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=SETIO_CALL_TIMEOUT_S)
        res = fut.result()
        if res is None:
            self._node.get_logger().warn(
                f"OnRobotToolIOGrip: set_io call timed out (pin={pin}, state={state})")
            return False
        return bool(res.success)

    def close_blocking(self, timeout_s: float = GRIP_DETECT_TIMEOUT_S) -> Tuple[bool, str]:
        """Send the CLOSE command and block until the gripper has had time
        to actuate. Returns (set_io_success, reason).

        NOTE: doesn't actually verify grip — see module docstring for the
        grip-detect TODO. In practice, sleeping for 1.5 s is enough for
        the RG6 to complete its close motion at default speed.
        """
        ok = self._call(PIN_TOOL_DOUT0, STATE_ON)
        if not ok:
            return False, "set_io close (pin=16, state=ON) failed"
        time.sleep(timeout_s)
        return True, "close commanded + settle"

    def open(self) -> Tuple[bool, str]:
        """Send the OPEN command. Non-blocking — does not wait for actuation."""
        ok = self._call(PIN_TOOL_DOUT0, STATE_OFF)
        if not ok:
            return False, "set_io open (pin=16, state=OFF) failed"
        return True, "open commanded"


# Standalone test harness — useful when you want to verify the helper
# without running the full pickplace. Usage:
#     python3 tests/onrobot_io_grip.py close
#     python3 tests/onrobot_io_grip.py open
#     python3 tests/onrobot_io_grip.py cycle    # close → wait → open
def _main():
    import sys
    if len(sys.argv) < 2 or sys.argv[1] not in ("close", "open", "cycle"):
        print("usage: onrobot_io_grip.py {close|open|cycle}")
        sys.exit(2)
    cmd = sys.argv[1]

    rclpy.init()
    node = Node("onrobot_io_grip_test")
    g = OnRobotToolIOGrip(node)
    if not g.connect():
        rclpy.shutdown()
        sys.exit(3)

    if cmd == "close":
        ok, msg = g.close_blocking()
        print(f"CLOSE: ok={ok} ({msg})")
    elif cmd == "open":
        ok, msg = g.open()
        print(f"OPEN:  ok={ok} ({msg})")
    elif cmd == "cycle":
        ok, msg = g.close_blocking()
        print(f"CLOSE: ok={ok} ({msg})")
        time.sleep(1.0)
        ok, msg = g.open()
        print(f"OPEN:  ok={ok} ({msg})")
    rclpy.shutdown()


if __name__ == "__main__":
    _main()

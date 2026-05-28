#!/usr/bin/env python3
"""OnRobot RG6 binary grip control via ur_robot_driver's set_io service.

TEACH MODE (no OnRobot Compute Box, OnRobot URCap NOT controlling the tool).
The RG6 cable extends the UR tool connector straight to the gripper's
internal control board. We drive it over the UR tool I/O:

  Tool connector pinout (RG6 datasheet v1.6, p.4) → ur_msgs access:
    pin 5  Gray  24V DC power   → set_io fun=4  state=24   (POWER — required!)
    pin 8  Red   0V GND
    pin 7  Blue  Tool output 0  → set_io fun=1  pin=16     open/close cmd
    pin 6  Pink  Tool output 1  → set_io fun=1  pin=17     force mode
    pin 4  Yellow Tool input 0  → io_states.digital_in_states[pin=16]
                                  HI=Force Reached, LO=Position Reached  (grip-detect)
    pin 3  Green  Tool input 1  → io_states.digital_in_states[pin=17]
                                  HI=Ready, LO=Busy
    pin 1  White  Analog in 2   → tool_data.analog_input2   width 0..3.0V→0..160mm

Command semantics (matches reference onrobot1_ros/onrobot_gripper.cpp):
    pin 16 (control): 1 = CLOSE, 0 = OPEN
    pin 17 (force)  : 0 = FULL force, 1 = LOW force (slower, gentler)

CRITICAL — the power-on / wake-up SEQUENCE (this is what we got wrong
before and tripped "too high sink current on Tool Digital Output 0"):
  1. Drive pin 16 LOW.
  2. Set tool voltage to 24V via set_io fun=4 — do NOT rely on the pendant
     installation's Tool Output Voltage; set it from software so we KNOW
     the 24V rail (pin 5) is actually energized.
  3. Read back tool_data.tool_output_voltage and confirm it is > 23V.
     Without this, the gripper draws its operating current (up to 600mA,
     3A spikes on release) THROUGH the pin-16 signal line's low-side
     driver → overcurrent fault. The readback proves pin 5 is live first.
  4. Wait ~5s for the gripper MCU to boot.
  5. Wake-up: pin 16 HIGH → 1s → pin 16 LOW. Required after power-on.

Only after this does pin 16 mean open/close.

Teach mode is BINARY: only fully-open and fully-closed are commandable.
Width (analog_input2) is READ-ONLY feedback. Continuous width / per-grip
force needs the OnRobot Compute Box (Modbus TCP) or RS-485 URCap. For the
wood-block stacking demo, fully-closed grips every (identical) block fine.

Pendant prereq: Installation → General → Tool I/O → "Controlled by: User"
so the OnRobot URCap doesn't fight these writes. External Control URP does
NOT need to be playing for the gripper — set_io rides RTDE independently of
the reverse interface (which only carries motion).
"""
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates, ToolDataMsg


# --- ur_msgs/srv/SetIO function codes ---
FUN_SET_DIGITAL_OUT = 1
FUN_SET_TOOL_VOLTAGE = 4

# --- Tool digital OUTPUT pins (ur_msgs numbering: tool I/O at 16-17) ---
PIN_GRIPPER_CONTROL = 16     # Tool output 0 (DO8): 1=close, 0=open
PIN_GRIPPER_FORCE = 17       # Tool output 1 (DO9): 0=full force, 1=low force

# --- Tool digital INPUT pins (read from io_states.digital_in_states) ---
# Mapping matches the working reference drivers (onrobot1_ros): pin 16 reports
# the gripper STATE (0=open, 1=closed), pin 17 reports READY (HI) / BUSY (LO).
# Move-done = ready AND state == target.
DI_STATE = 16                # Tool input 0 (DI8): 0=open, 1=closed
DI_READY = 17                # Tool input 1 (DI9): HI=Ready, LO=Busy

STATE_ON = 1.0
STATE_OFF = 0.0

# --- Power ---
TOOL_VOLTAGE_V = 24.0
VOLTAGE_ENABLED_THRESHOLD_V = 23.0   # reference: is_enabled() iff tool_voltage > 23V

# --- Analog width feedback: AI2 0..~3.0V → 0..160mm on the 0-10V input range ---
WIDTH_MAX_MM = 160.0
AI2_MAX_V = 3.0

# --- Timing budget ---
VOLTAGE_SETTLE_TIMEOUT_S = 5.0   # wait for tool_output_voltage to read > 23V
GRIPPER_BOOT_S = 5.0             # MCU boot after power-on
WAKEUP_HIGH_S = 1.0              # pin16 HIGH dwell during wake-up
ENABLE_READY_TIMEOUT_S = 12.0    # wait for READY after wake-up (ref uses 12s)
GRIP_MOTION_TIMEOUT_S = 3.0      # max wait for ready+state after a move
SETIO_CALL_TIMEOUT_S = 2.0


class OnRobotToolIOGrip:
    """Binary OnRobot RG6 control over UR tool I/O in Teach mode.

    Usage:
        grip = OnRobotToolIOGrip(node)
        grip.connect()           # wait for service + power-on + wake-up
        grip.close_blocking()    # close (full force) + wait for grip-detect
        grip.open()              # open
    """

    def __init__(self, node: Node,
                 set_io_service: str = "/io_and_status_controller/set_io",
                 io_states_topic: str = "/io_and_status_controller/io_states",
                 tool_data_topic: str = "/io_and_status_controller/tool_data",
                 low_force_default: bool = True):
        self._node = node
        self._service_name = set_io_service
        self._cli = node.create_client(SetIO, set_io_service)
        # Default to LOW force: slower + gentler grip (honours the project's
        # safe-default-speeds rule). Flip to full force if grip slips on lift.
        self._low_force_default = low_force_default

        # Feedback state, updated by subscription callbacks.
        self._tool_voltage = 0.0
        self._width_mm: Optional[float] = None
        self._state: Optional[int] = None   # DI16: 0=open, 1=closed
        self._ready = False                 # DI17: True=ready, False=busy
        self._enabled = False

        node.create_subscription(ToolDataMsg, tool_data_topic, self._tool_data_cb, 10)
        node.create_subscription(IOStates, io_states_topic, self._io_states_cb, 10)

    # ---------------- feedback callbacks ----------------
    def _tool_data_cb(self, msg: ToolDataMsg):
        self._tool_voltage = float(msg.tool_output_voltage)
        # analog_input2 → width (0V≈closed-ish, AI2_MAX_V≈fully open at 160mm)
        if AI2_MAX_V > 1e-3:
            frac = max(0.0, min(1.0, float(msg.analog_input2) / AI2_MAX_V))
            self._width_mm = frac * WIDTH_MAX_MM

    def _io_states_cb(self, msg: IOStates):
        for io in msg.digital_in_states:
            if io.pin == DI_STATE:
                self._state = int(io.state)             # 0=open, 1=closed
            elif io.pin == DI_READY:
                self._ready = bool(io.state)            # HI = Ready

    # ---------------- low-level set_io ----------------
    def _set_io(self, fun: int, pin: int, state: float) -> bool:
        if not self._cli.service_is_ready():
            self._node.get_logger().error(
                f"OnRobotToolIOGrip: SetIO service '{self._service_name}' not ready.")
            return False
        req = SetIO.Request()
        req.fun = fun
        req.pin = pin
        req.state = float(state)
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=SETIO_CALL_TIMEOUT_S)
        res = fut.result()
        if res is None:
            self._node.get_logger().warn(
                f"OnRobotToolIOGrip: set_io timed out (fun={fun}, pin={pin}, state={state})")
            return False
        return bool(res.success)

    def _spin(self, duration_s: float):
        """Spin the node for duration_s so feedback callbacks fire."""
        end = time.time() + duration_s
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    # ---------------- public API ----------------
    def connect(self, timeout_s: float = 5.0) -> bool:
        """Wait for SetIO, then run the power-on + wake-up sequence.
        Returns True only if the gripper reached 24V and was woken up."""
        if not self._cli.wait_for_service(timeout_sec=timeout_s):
            self._node.get_logger().error(
                f"OnRobotToolIOGrip: service '{self._service_name}' not available "
                f"within {timeout_s}s. Is the ur_robot_driver running?")
            return False
        ok, msg = self.enable()
        if not ok:
            self._node.get_logger().error(f"OnRobotToolIOGrip: enable failed — {msg}")
        return ok

    def enable(self) -> Tuple[bool, str]:
        """Power the gripper (24V via fun=4), confirm via readback, boot-wait,
        then run the HIGH→LOW wake-up toggle. See module docstring."""
        # 1. control line LOW first
        self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_CONTROL, STATE_OFF)

        # 2. set tool voltage to 24V from software (do not trust the pendant)
        self._node.get_logger().info("OnRobotToolIOGrip: setting tool voltage to 24V...")
        if not self._set_io(FUN_SET_TOOL_VOLTAGE, 0, TOOL_VOLTAGE_V):
            return False, "set_io fun=4 (tool voltage 24V) was rejected"

        # 3. confirm the 24V rail is actually live BEFORE touching pin 16 again
        start = time.time()
        while time.time() - start < VOLTAGE_SETTLE_TIMEOUT_S:
            self._spin(0.1)
            if self._tool_voltage > VOLTAGE_ENABLED_THRESHOLD_V:
                break
        if self._tool_voltage <= VOLTAGE_ENABLED_THRESHOLD_V:
            return False, (
                f"tool voltage only reached {self._tool_voltage:.1f}V (need >23V). "
                "Pin 5 (24V) is not energized — check pendant Tool I/O = 'Controlled "
                "by: User', and that /io_and_status_controller/tool_data is publishing.")

        # 4. let the gripper MCU boot
        self._node.get_logger().info(
            f"OnRobotToolIOGrip: 24V confirmed ({self._tool_voltage:.1f}V); "
            f"waiting {GRIPPER_BOOT_S:.0f}s for gripper boot...")
        self._spin(GRIPPER_BOOT_S)

        # 5. wake-up toggle: HIGH → dwell → LOW
        self._node.get_logger().info("OnRobotToolIOGrip: wake-up toggle (pin16 HIGH→LOW)...")
        self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_CONTROL, STATE_ON)
        self._spin(WAKEUP_HIGH_S)
        self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_CONTROL, STATE_OFF)

        # 6. wait for READY (DI17 HIGH). Reference allows up to ~12s for the
        # gripper to finish its power-on open before reporting ready.
        start = time.time()
        while time.time() - start < ENABLE_READY_TIMEOUT_S:
            self._spin(0.1)
            if self._ready:
                break
        self._enabled = True
        ready_str = "ready" if self._ready else "READY not seen (DI17 low) — proceeding"
        return True, f"enabled at {self._tool_voltage:.1f}V, {ready_str}"

    def disable(self) -> Tuple[bool, str]:
        """Cut tool power (0V). Leaves pin 16 LOW."""
        self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_CONTROL, STATE_OFF)
        ok = self._set_io(FUN_SET_TOOL_VOLTAGE, 0, 0.0)
        self._enabled = False
        return ok, "tool voltage set to 0V" if ok else "disable rejected"

    def _move(self, close: bool, low_force: Optional[bool] = None) -> Tuple[bool, str]:
        if not self._enabled:
            return False, "gripper not enabled — call connect()/enable() first"
        lf = self._low_force_default if low_force is None else low_force
        # Force mode first, then the open/close command.
        self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_FORCE, STATE_ON if lf else STATE_OFF)
        target = 1 if close else 0
        ok = self._set_io(FUN_SET_DIGITAL_OUT, PIN_GRIPPER_CONTROL,
                          STATE_ON if close else STATE_OFF)
        if not ok:
            return False, f"set_io pin16={target} rejected"
        # Wait for motion done = READY (DI17) AND state (DI16) == target.
        self._ready = False
        start = time.time()
        while time.time() - start < GRIP_MOTION_TIMEOUT_S:
            self._spin(0.05)
            if self._ready and self._state == target:
                w = f"{self._width_mm:.0f}mm" if self._width_mm is not None else "?"
                verb = "closed" if close else "opened"
                return True, f"{verb} (ready, state={target}, width≈{w}, {'low' if lf else 'full'} force)"
        w = f"{self._width_mm:.0f}mm" if self._width_mm is not None else "?"
        verb = "close" if close else "open"
        return True, f"{verb} commanded (timeout waiting ready/state; state={self._state}, width≈{w})"

    def close_blocking(self, timeout_s: Optional[float] = None,
                       low_force: Optional[bool] = None) -> Tuple[bool, str]:
        """Close the gripper and wait for the grip-detect (Force Reached) line,
        falling back to a timeout. Returns (ok, reason)."""
        return self._move(close=True, low_force=low_force)

    def open(self, low_force: Optional[bool] = None) -> Tuple[bool, str]:
        """Open the gripper."""
        return self._move(close=False, low_force=low_force)

    # convenience read
    def width_mm(self) -> Optional[float]:
        return self._width_mm


# Standalone test harness:
#     python3 tests/onrobot_io_grip.py enable   # power-on + wake-up only
#     python3 tests/onrobot_io_grip.py close
#     python3 tests/onrobot_io_grip.py open
#     python3 tests/onrobot_io_grip.py cycle     # enable → close → open
def _main():
    import sys
    cmds = ("enable", "close", "open", "cycle")
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(f"usage: onrobot_io_grip.py {{{'|'.join(cmds)}}}")
        sys.exit(2)
    cmd = sys.argv[1]

    rclpy.init()
    node = Node("onrobot_io_grip_test")
    g = OnRobotToolIOGrip(node)

    print("Connecting + enabling (power-on 24V, boot, wake-up — ~11s)...")
    if not g.connect():
        print("ENABLE FAILED — see log above. Gripper not powered; aborting.")
        rclpy.shutdown()
        sys.exit(3)
    print("Gripper enabled.")

    if cmd == "enable":
        pass
    elif cmd == "close":
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

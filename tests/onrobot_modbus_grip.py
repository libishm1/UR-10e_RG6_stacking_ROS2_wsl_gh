#!/usr/bin/env python3
"""OnRobot RG6 control over the UR tool-flange RS485 (Modbus RTU).

This is the CHOSEN real-hardware gripper path (the digital tool-I/O path is
parked — it tripped tool-connector overcurrent; see
wiki/rg6_urcap_hardware_pitfalls.md). RS485 is the RG6's native channel — the
same one the OnRobot URCap uses — so there is NO overcurrent risk, and we get
continuous width + force + grip-detect.

Bridge: launch ur_robot_driver with use_tool_communication:=true (see
scripts/launch_real_rs485.sh). The driver exposes the tool RS485 as a host
pseudo-serial device (default /tmp/ttyUR via socat). This client opens it.

Why plain pyserial + hand-rolled Modbus RTU (not pymodbus): the socat pty
rejects pymodbus's serial open (it uses exclusive=True + modem-control ioctls
that a pty doesn't support → "(22) Invalid argument" on open). Plain pyserial
with rtscts=False, dsrdtr=False, exclusive=False opens the pty cleanly, and
Modbus RTU framing is trivial. Verified 2026-05-28. No pymodbus dependency.

Register map (from Osaka-University-Harada-Laboratory/onrobot; see
wiki/rg6_rs485_modbus.md). Modbus unit/device id = 65 (single Quick Changer):

  WRITE holding registers @ 0  (3 words):
    reg 0: target force  — 0.1 N units   (RG6: 0..1200 = 0..120.0 N)
    reg 1: target width  — 0.1 mm units  (RG6: 0..1600 = 0..160.0 mm)
    reg 2: control word  — 1=grip(move to target), 8=stop, 16=grip+offset
  READ holding registers @ 258 (18 words):
    word 9 : actual width (0.1 mm)
    word 10: status bitfield — bit0=busy, bit1=grip-detected, bits2-5=safety

NOTE (verify at the cell): the addresses above come from the OnRobot
Modbus-TCP/Compute-Box library. The Compute Box is believed to be a
transparent TCP↔RTU gateway so the same map applies over the tool RS485,
but confirm by reading actual width (reg 258+9) and checking it tracks the
real finger position before trusting it.

socat note: the driver's socat uses `waitslave`, so OPEN THE PORT ONCE and
keep it open for the whole session (connect() does this). Repeatedly
opening/closing the pty can degrade socat — connect once, reuse the handle.
"""
import struct
import time
from typing import List, Optional, Tuple

try:
    import serial  # pyserial (already present; no pymodbus needed)
    _HAVE_SERIAL = True
    _SERIAL_ERR = None
except Exception as e:
    _HAVE_SERIAL = False
    _SERIAL_ERR = e


# --- connection ---
DEFAULT_PORT = "/tmp/ttyUR"
DEVICE_ID = 65               # single Quick Changer; verify if dual/other
# pty nominal baud is cosmetic (socat passes raw bytes; the real 1M RS485 baud
# is on the cabinet). A pty can reject 1000000; 115200 always opens.
SERIAL_BAUD = 115200
SERIAL_PARITY = "E"          # even
SERIAL_STOPBITS = 1
SERIAL_BYTESIZE = 8
SERIAL_TIMEOUT_S = 1.0

# --- Modbus function codes ---
FN_READ_HOLDING = 0x03
FN_WRITE_MULTIPLE = 0x10

# --- register map ---
# VERIFIED ON HARDWARE 2026-05-28 (unit 65, read @258):
#   - WRITE @0 [force*10, width*10, 1] physically moves the gripper. CONFIRMED.
#   - COMMANDS WORK: cmd 160 -> full open (~150 mm, mechanical max per tape);
#     cmd 0 -> fully closed (0 mm per tape). So open()=160 / close_blocking()=0
#     are the calibrated demo values. With the ~50 mm wood block present, a
#     close-to-0 at force stops on the block and holds.
#   - offset 9 (reg 267) TRACKS position but is NON-LINEAR vs the physical
#     fingertip gap (the RG6 fingers PIVOT, so the internal metric != gap):
#       cmd 160 / 150 mm tape -> reg 160.2
#       cmd 0   /   0 mm tape -> reg  64.6
#     i.e. reg ranges ~65 (closed) .. ~160 (open) over a 0..150 mm physical
#     range. DO NOT treat reg 267 as exact mm — use it for relative feedback
#     only. Also: before the FIRST grip command of a power cycle it reads
#     garbage (~64404 -> "6440 mm"); valid only after a grip.
#   - offset 5 is NOT width (ruled out earlier).
# GRIP-DETECT VERIFIED ON HARDWARE 2026-05-28 (status word @258 offset 10):
#   close on a BLOCK   -> offset 10 == 1 (bit0)  = object gripped (force reached)
#   close on NOTHING   -> offset 10 == 2 (bit1)  = position reached, no object
# (controlled close-on-block vs close-on-empty at the same position cancels the
#  position noise.) This matches OnRobot's "Force Reached vs Position Reached"
#  DI feedback. So grip-detect = offset-10 bit0. NOTE bit0 can also be set
#  transiently during motion, so check grip_detected() AFTER the move settles.
CMD_ADDR = 0                 # write [force, width, control]
STATUS_ADDR = 258            # read 18 words
STATUS_COUNT = 18
ST_WIDTH = 9                 # reg 267 = actual width (0.1 mm) — VERIFIED (non-linear)
ST_STATUS = 10               # status bitfield — VERIFIED (grip-detect, see above)
BIT_GRIP_DETECT = 1 << 0     # bit0: object gripped / force reached
BIT_POSITION_REACHED = 1 << 1  # bit1: reached commanded position, no object
BIT_GRIP_DETECT = 1 << 1

CTRL_GRIP = 1
CTRL_STOP = 8

# --- RG6 limits + defaults ---
RG6_MAX_FORCE_N = 120.0
RG6_MAX_WIDTH_MM = 160.0
RG6_MIN_WIDTH_MM = 0.0
# Safe/slow default: lower force = slower close (RG6 speed proportional to
# force). Honours the project safe-default rule; bump only on explicit ask.
DEFAULT_FORCE_N = 40.0


def _crc16(data: bytes) -> int:
    """Modbus RTU CRC-16 (poly 0xA001), returned as int (append little-endian)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class OnRobotModbusGrip:
    """Continuous-width OnRobot RG6 control over tool RS485 (Modbus RTU).

    Drop-in interface match with OnRobotToolIOGrip: connect(), close_blocking(),
    open(). Adds grip_to(width, force) for true continuous control.

    `node` is optional and only used for logging (this client talks serial
    Modbus directly — it is NOT a ROS node and uses no ROS topics/services).
    """

    def __init__(self, node=None, port: str = DEFAULT_PORT,
                 device_id: int = DEVICE_ID,
                 default_force_n: float = DEFAULT_FORCE_N):
        self._node = node
        self._port = port
        self._device_id = device_id
        self._default_force_n = default_force_n
        self._ser = None

    def _log(self, msg: str, level: str = "info"):
        if self._node is not None:
            getattr(self._node.get_logger(), level)(msg)
        else:
            print(msg)

    # ---------- connection ----------
    def connect(self, timeout_s: float = 5.0) -> bool:
        if not _HAVE_SERIAL:
            self._log(f"OnRobotModbusGrip: pyserial unavailable ({_SERIAL_ERR}).", "error")
            return False
        try:
            # exclusive=False + no modem-control flow so the socat pty accepts it.
            self._ser = serial.Serial(
                port=self._port, baudrate=SERIAL_BAUD, parity=SERIAL_PARITY,
                stopbits=SERIAL_STOPBITS, bytesize=SERIAL_BYTESIZE,
                timeout=SERIAL_TIMEOUT_S, rtscts=False, dsrdtr=False,
                exclusive=False)
        except Exception as e:
            self._log(f"OnRobotModbusGrip: cannot open '{self._port}' ({e}). Is the "
                      "driver up with use_tool_communication:=true and 54321 open "
                      "(scripts/launch_real_rs485.sh)?", "error")
            return False
        regs = self._read_status()
        if regs is None:
            self._log(f"OnRobotModbusGrip: port open but no Modbus reply from "
                      f"device_id={self._device_id}. Check tool RS485, unit id, and "
                      "that 54321 is reachable.", "error")
            return False
        self._log(f"OnRobotModbusGrip: connected on {self._port} "
                  f"(device_id={self._device_id}), width={regs[ST_WIDTH]/10.0:.1f}mm")
        return True

    def disconnect(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ---------- Modbus RTU framing ----------
    def _txn(self, request: bytes, expected_len: int) -> Optional[bytes]:
        """Send a framed request (sans CRC), append CRC, read expected_len bytes,
        validate CRC + function code. Returns the full response or None."""
        if self._ser is None:
            return None
        frame = request + struct.pack("<H", _crc16(request))
        try:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            resp = self._ser.read(expected_len)
        except Exception as e:
            self._log(f"OnRobotModbusGrip: serial txn error {e}", "warn")
            return None
        if len(resp) < 4:
            return None
        # CRC check
        if _crc16(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
            self._log("OnRobotModbusGrip: CRC mismatch in response", "warn")
            return None
        # Modbus exception (function code | 0x80)
        if resp[1] & 0x80:
            self._log(f"OnRobotModbusGrip: modbus exception code {resp[2]}", "warn")
            return None
        return resp

    def _read_status(self) -> Optional[List[int]]:
        # Read STATUS_COUNT holding regs from STATUS_ADDR.
        req = struct.pack(">BBHH", self._device_id, FN_READ_HOLDING,
                          STATUS_ADDR, STATUS_COUNT)
        # response: id, fn, bytecount, data(2*count), crc(2)
        expected = 3 + STATUS_COUNT * 2 + 2
        resp = self._txn(req, expected)
        if resp is None or len(resp) < expected:
            return None
        byte_count = resp[2]
        if byte_count != STATUS_COUNT * 2:
            return None
        regs = list(struct.unpack(">" + "H" * STATUS_COUNT, resp[3:3 + byte_count]))
        return regs

    def _write_grip(self, width_mm: float, force_n: float) -> bool:
        w = int(round(max(RG6_MIN_WIDTH_MM, min(RG6_MAX_WIDTH_MM, width_mm)) * 10))
        f = int(round(max(0.0, min(RG6_MAX_FORCE_N, force_n)) * 10))
        values = [f, w, CTRL_GRIP]
        data = b"".join(struct.pack(">H", v) for v in values)
        req = struct.pack(">BBHHB", self._device_id, FN_WRITE_MULTIPLE,
                          CMD_ADDR, len(values), len(data)) + data
        # response: id, fn, addr(2), count(2), crc(2) = 8 bytes
        resp = self._txn(req, 8)
        return resp is not None

    def _wait_done(self, settle_s: float = 2.0) -> Optional[List[int]]:
        # The RG6 completes a move in ~1 s. The status word has no clean "busy"
        # bit (bit0 = open-or-holding, bit1 = closed-empty), so settle then read.
        time.sleep(settle_s)
        return self._read_status()

    # ---------- read API ----------
    def read_width_mm(self) -> Optional[float]:
        regs = self._read_status()
        return None if regs is None else regs[ST_WIDTH] / 10.0

    def status_word(self) -> Optional[int]:
        regs = self._read_status()
        return None if regs is None else regs[ST_STATUS]

    def grip_detected(self) -> bool:
        """True if an object is held. Only meaningful right AFTER a close: a
        closed gripper reports bit0 (grip/force) when holding an object and
        bit1 (position reached) when it closed on nothing. When OPEN, bit0 is
        also set, so don't rely on this except just after close_blocking()."""
        regs = self._read_status()
        if regs is None:
            return False
        st = regs[ST_STATUS]
        return bool((st & BIT_GRIP_DETECT) and not (st & BIT_POSITION_REACHED))

    # ---------- motion API ----------
    def grip_to(self, width_mm: float, force_n: Optional[float] = None) -> Tuple[bool, str]:
        """Move to target width applying up to target force, then block until
        the gripper reports not-busy (or timeout). Returns (ok, reason)."""
        f = self._default_force_n if force_n is None else force_n
        if not self._write_grip(width_mm, f):
            return False, "modbus write failed"
        regs = self._wait_done()
        if regs is None:
            return True, f"commanded width={width_mm:.0f}mm (no status readback)"
        w = regs[ST_WIDTH] / 10.0
        st = regs[ST_STATUS]
        det = bool((st & BIT_GRIP_DETECT) and not (st & BIT_POSITION_REACHED))
        return True, f"width={w:.1f}mm grip_detected={det} (status={st}) force={f:.0f}N"

    # ---------- drop-in compatibility with OnRobotToolIOGrip ----------
    def close_blocking(self, width_mm: float = 0.0,
                       force_n: Optional[float] = None) -> Tuple[bool, str]:
        """Close to target width (default 0 = grip on object) at force."""
        return self.grip_to(width_mm, force_n)

    def open(self, width_mm: float = RG6_MAX_WIDTH_MM,
             force_n: Optional[float] = None) -> Tuple[bool, str]:
        """Open to target width (default fully open)."""
        return self.grip_to(width_mm, force_n)


# Standalone harness (driver must be up with use_tool_communication + 54321 open):
#   python3 tests/onrobot_modbus_grip.py status
#   python3 tests/onrobot_modbus_grip.py close [width_mm] [force_n]
#   python3 tests/onrobot_modbus_grip.py open  [width_mm]
#   python3 tests/onrobot_modbus_grip.py cycle
def _main():
    import sys
    cmds = ("status", "close", "open", "cycle")
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(f"usage: onrobot_modbus_grip.py {{{'|'.join(cmds)}}} [args]")
        sys.exit(2)
    cmd = sys.argv[1]

    g = OnRobotModbusGrip()
    if not g.connect():
        print("CONNECT FAILED — see message above.")
        sys.exit(3)

    if cmd == "status":
        print(f"width={g.read_width_mm()} mm  status_word={g.status_word()}  "
              f"grip_detected={g.grip_detected()}")
    elif cmd == "close":
        w = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        f = float(sys.argv[3]) if len(sys.argv) > 3 else None
        ok, msg = g.close_blocking(w, f)
        print(f"CLOSE->{w:.0f}mm: ok={ok} ({msg})")
    elif cmd == "open":
        w = float(sys.argv[2]) if len(sys.argv) > 2 else RG6_MAX_WIDTH_MM
        ok, msg = g.open(w)
        print(f"OPEN->{w:.0f}mm: ok={ok} ({msg})")
    elif cmd == "cycle":
        ok, msg = g.close_blocking(0.0)
        print(f"CLOSE: ok={ok} ({msg})")
        time.sleep(1.0)
        ok, msg = g.open()
        print(f"OPEN:  ok={ok} ({msg})")

    g.disconnect()


if __name__ == "__main__":
    _main()

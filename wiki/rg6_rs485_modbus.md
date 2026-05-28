# RG6 over RS485/Modbus (tool flange) — the chosen ROS 2 gripper path

## Purpose

Implementation reference for controlling the OnRobot RG6 from ROS 2 over the
**UR tool-flange RS485** (Modbus), bridged to the host by
`ur_robot_driver`'s tool-communication feature. This is the path chosen
after the digital tool-I/O approach failed
([rg6_urcap_hardware_pitfalls.md](rg6_urcap_hardware_pitfalls.md)). It uses
the gripper's **native** control channel — the same RS485 the OnRobot URCap
uses — so there is **no overcurrent risk** and it gives **continuous width
and force**, not just open/close.

## Confirmed facts — tool communication bridge

The UR e-Series tool connector has RS485 **built in** (no extra hardware).
`ur_robot_driver` bridges it to a host pseudo-serial device. Launch args
(from the UR ROS driver `setup_tool_communication` doc):

| Arg | Value for this cell | Meaning |
|---|---|---|
| `use_tool_communication` | `true` | enable the bridge |
| `tool_voltage` | `24` | power the RG6 |
| `tool_parity` | `2` | even |
| `tool_baud_rate` | `1000000` | 1M baud |
| `tool_stop_bits` | `1` | one stop bit |
| `tool_rx_idle_chars` | `1.5` | matches pendant |
| `tool_tx_idle_chars` | `3.5` | matches pendant |
| `tool_device_name` | `/tmp/ttyUR` | host pty the Modbus client opens |

These match the OnRobot URCap's own RS485 settings (1M / Even / One /
1.5 / 3.5) seen on the pendant Communication Interface screen — strong
confirmation this is the right channel. Reference:
[tonydle/ur_onrobot](https://github.com/tonydle/ur_onrobot).

A Modbus-RTU client opens `/tmp/ttyUR` and talks to the gripper.

**HOST DEPENDENCY — `socat` (verified 2026-05-28).** The driver's
`tool_communication.py` shells out to **`socat`** to turn the cabinet's
tool-comm TCP stream into the `/tmp/ttyUR` pty:

```
socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:192.168.1.100:54321
```

If `socat` is missing the node dies with `FileNotFoundError: 'socat'` and
`/tmp/ttyUR` never appears. Install once: `sudo apt-get install -y socat`.
`scripts/launch_real_rs485.sh` now preflight-checks for it.

**CORRECTION — the rs485 daemon URCap IS required (re-verified 2026-05-28).**
An earlier note here claimed e-Series needs no rs485 URCap; that was wrong.
socat *starting* is not the same as port 54321 being *open* — and with no
rs485 URCap, **54321 is REFUSED** (`</dev/tcp/192.168.1.100/54321` fails),
even with `use_tool_communication:=true`, the `ros` installation loaded, and
`external_control.urp` playing. The thing that actually opens 54321 and
"exposes the tool communication device to the network" is the **rs485 daemon
URCap** (UR doc `setup_tool_communication.md`). The bundle ships in the
driver: `ur_robot_driver/resources/rs485-1.0.urcap`. Installed 2026-05-28 by
SCP'ing it to the cabinet's `/root/.urcaps/rs485-1.0.jar` (alongside the
OnRobot + External Control bundles); requires a PolyScope restart.

**OPEN — does it coexist with the OnRobot URCap?** Both claim the tool RS485
bus; UR docs note such conflicts (robotiq URCap blocks rs485 URCap). Testing
whether the rs485 daemon activates and opens 54321 with the OnRobot URCap
still installed (device=None). If they conflict, the OnRobot URCap must be
uninstalled to use the ROS gripper.

## Confirmed facts — OnRobot RG Modbus register map

Sourced from
[Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot)
(`baseOnRobotRG.py`, `comModbusTcp.py`). The same register addresses apply
over RTU (the Compute Box is a transparent TCP↔RTU gateway), but **this must
be runtime-verified** over the tool RS485 (see open questions).

**Modbus unit / slave id:** `65` (single Quick Changer).

**Command — write holding registers starting at address `0`** (3 words):

| Reg | Field | RG6 range | Units |
|---|---|---|---|
| 0 | target **force** (`rGFR`) | 0–1200 | 0.1 N (1200 = 120.0 N) |
| 1 | target **width** (`rGWD`) | 0–1600 | 0.1 mm (1600 = 160.0 mm) |
| 2 | **control** (`rCTR`) | — | `1`=grip (move to target), `8`=stop, `16`=grip-with-offset |

So a grip = write `[force*10, width*10, 1]` to registers 0–2 at unit 65.

**Status — read holding registers at address `258`, count `18`:**

| Word offset | Field | Meaning |
|---|---|---|
| 0 | `gFOF` | finger/object detection |
| 9 | `gGWD` | **actual width** (0.1 mm) |
| 10 | `gSTA` | **status word** (bitfield: busy, grip-detected, S1/S2 safety) |
| 17 | `gWDF` | width-with-default |

`gSTA` bitfield (OnRobot RG convention): bit0 = busy (moving), bit1 = grip
detected (object held), bits 2–5 = S1/S2 safety triggered/pushed.

## Working assumptions (VERIFY before trusting)

1. **Register map holds over RTU.** The addresses above are from the
   Modbus-**TCP** (Compute Box) library. The Compute Box is believed to be a
   transparent gateway so the gripper's native RTU registers match — but
   confirm by reading `actual width` (reg 258+9) over `/tmp/ttyUR` and
   checking it tracks the real finger position.
2. ~~e-Series tool-comm URCap requirement.~~ **RESOLVED 2026-05-28: no rs485
   URCap needed** — the driver connects natively to the controller's tool-comm
   port 54321. Only host dependency is `socat` (see above).
3. **URCap coexistence — CONFIRMED 2026-05-28: the OnRobot-default
   installation blocks the bridge.** On that installation the cabinet's
   tool-comm port **54321 is REFUSED** (`</dev/tcp/192.168.1.100/54321`
   fails). socat still creates `/tmp/ttyUR`, but with nothing to bridge the
   Modbus open/read fails. Root cause: the OnRobot URCap owns the RS485 bus,
   so the controller won't forward tool comm to 54321. To open 54321, load
   the `ros` installation: Tool I/O **Controlled by User** + **Communication
   Interface** + OnRobot Setup **device None**. (Likely also needs
   external_control.urp PLAYING so the driver's tool-comm forwarding is
   active — verify.) Keep the OnRobot URCap for the `onrobot` installation
   (two-installation pattern).

4. **socat pty baud is cosmetic.** A fresh pty in socat `waitslave` state
   rejects baud 1000000 (EINVAL); the client uses 115200 instead — the real
   1M baud is on the cabinet's RS485 side. socat passes raw bytes, so the
   pty's nominal baud doesn't affect the wire. Once socat is idle all bauds
   open; the transient EINVAL appears only while socat retries a refused
   54321 connection.

## Implementation plan

1. **Driver relaunch** with the tool-comm args above (extend
   `scripts/launch_real.sh` or a new `launch_real_rs485.sh`).
2. **`tests/onrobot_modbus_grip.py`** — new Modbus-RTU helper (pymodbus),
   open `/tmp/ttyUR`, unit 65:
   - `set_force_n(f)`, `set_width_mm(w)`, `grip()` → write regs 0–2.
   - `read_width_mm()`, `is_busy()`, `grip_detected()` → read reg 258+.
   - `open()` = grip to max width; `close()` = grip to min width at force.
3. **Wire into `play_pickplace.py`** `--real-gripper` (swap the parked
   `onrobot_io_grip.py` for the Modbus helper).
4. **Test** (low risk — RS485 is a comm bus, cannot overcurrent the tool
   driver): read actual width first to validate the map, then a slow
   close/open, then a 1-cycle pickplace.

## Sources

- [tonydle/ur_onrobot](https://github.com/tonydle/ur_onrobot) — e-Series ROS
  OnRobot driver; tool-comm params; depends on the Osaka library.
- [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot)
  — register map (`baseOnRobotRG.py`, `comModbusTcp.py`).
- UR ROS driver `setup_tool_communication.md` — bridge args + URCap note.

## Last updated

2026-05-28 (research only — not yet implemented or hardware-verified).

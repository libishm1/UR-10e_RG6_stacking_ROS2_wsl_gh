# RG6 + OnRobot URCap — hardware pitfalls and the digital-control handoff

## Purpose

A "burnt by this" record of the **2026-05-28 cell session** where we tried
to drive the RG6 over UR **tool digital I/O** (`set_io` pins 16/17) and hit
repeated **tool-connector overcurrent faults**. The gripper never moved.

This page exists so the next attempt does not repeat the same fault-trip
cycle, captures every URCap-handling gotcha we found, and hands off a clean
procedure for retrying the digital path (with the leading hypothesis that
the OnRobot **URCap must be fully uninstalled** first).

See also: [rg6_control_mechanisms.md](rg6_control_mechanisms.md) (the A/B/C
mechanism comparison), [decisions.md](decisions.md) (locked decisions).

---

## Confirmed facts — the faults we hit (2026-05-28)

All three faults are **tool-connector overcurrent** trips. The gripper
**never actuated** in any digital attempt. The 24 V power rail itself was
fine throughout (`tool_data.tool_output_voltage` read 24.0 V, no power fault).

| # | Time | Fault code | Message | Pendant config at the time |
|---|---|---|---|---|
| 1 | 12:15 | **C289A2** | "Too high **sink** current detected on Digital Output: **0**, **low** side. Tool Digital Output pin has been turned off…" | Tool I/O = User; DO0/DO1 = **Sinking (NPN)**; 24 V; first naive `set_io` (no power-on sequence) |
| 2 | 12:59 | **C289A1** | "Too high current detected on Digital Output: **1**, **high** side… too high **source** current" | DO0/DO1 switched to **Sourcing (PNP)**; new helper with proper 24 V power-on + wake-up |
| 3 | 13:00 | **C289A1** | Same as #2, **recurring at every initialization** | DO1 left **latched ON** from the previous `set_io`; re-driven on each power-up |

**The decisive observation:** sinking trips on DO0 (low side), sourcing
trips on DO1 (high side). **Both polarities overcurrent.** That is not a
polarity bug — it means the RG6's tool digital lines present a **low-impedance
load** (the gripper's own control board is holding them), so driving them in
either direction shorts into that load and trips the cabinet's protection.

---

## Root-cause analysis — why the digital path failed

**The RG6 was not in "Teach mode."** OnRobot RG2/RG6 grippers have two
control modes:

- **URCap / RS485 mode (default, what this cell ships in):** the gripper is
  commanded over the tool **RS485** bus (Modbus), driven by the OnRobot
  URCap. The gripper's MCU owns the tool digital I/O lines.
- **Teach mode:** the gripper accepts simple **digital I/O** open/close +
  force, two positions only. This is what the reference drivers
  (`onrobot1_ros`) assume — their README says verbatim *"assumes a gripper
  in Teach mode **(without the UR Caps OnRobot installed)**."*

On this cell the OnRobot URCap is installed and the gripper sits in RS485
mode, so its MCU holds DO8/DO9. When we drove `set_io` pins 16/17, we fought
the MCU → overcurrent, regardless of sinking/sourcing. **The digital path
cannot work until the gripper is actually in Teach mode**, and we could not
find a software way to switch it while the URCap was installed.

This corroborates — on hardware — the binary-only caveat already in
[decisions.md](decisions.md): the directly-mounted RG6 is fundamentally an
**RS485/Modbus** device.

---

## Confirmed I/O mapping (RG6 datasheet v1.6 p.4 + ur_msgs)

Keep this regardless of which control path we end up on:

| RG6 pin | Wire | e-Series name | ur_msgs access | Function |
|---|---|---|---|---|
| 5 | Gray | 24V DC | `set_io fun=4 state=24` | Power (10–24–26 V, **typ. 24 V**) |
| 8 | Red | 0V GND | — | Ground |
| 7 | Blue | Tool output 0 | `set_io fun=1 pin=16` | Open/close (1=close, 0=open) |
| 6 | Pink | Tool output 1 | `set_io fun=1 pin=17` | Force mode (0=full, 1=low) |
| 4 | Yellow | Tool input 0 (DI8) | `io_states.digital_in_states[16]` | Position/Force reached |
| 3 | Green | Tool input 1 (DI9) | `io_states.digital_in_states[17]` | Busy(LO)/Ready(HI) |
| 1 | White | Analog input 2 | `tool_data.analog_input2` | Width 0–3 V → 0–160 mm |
| 2 | Brown | Analog input 3 | `tool_data.analog_input3` | (unused) |

Feedback semantics from the working reference
([gripper_controller.py:158](../src/onrobot1_ros/onrobot_driver/onrobot_driver/gripper_controller.py#L158)):
`digital_in_states[16]` = state (0=open/1=closed), `digital_in_states[17]`
= ready. Move-done = `ready AND state == target`.

---

## URCap-handling gotchas (do NOT relearn these the hard way)

1. **Tool I/O ownership.** Default is **Controlled by: OnRobot** (URCap owns
   the tool interface; user options are overridden). For any ROS-side tool
   control set **Installation → General → Tool I/O → Controlled by: User**.

2. **The URP won't play: "onrobot setup is not correct".** With Tool I/O =
   User, the OnRobot URCap's installation node fails validation and **blocks
   `external_control.urp` from playing.** Workaround that worked:
   **Installation → URCaps → OnRobot Setup → Device = "No connection"**.
   The URP then plays. (This does NOT enable digital control — it only
   stops the URCap from blocking the program.)

3. **`set_io` does NOT need the URP playing.** Tool voltage (`fun=4`) and
   digital out (`fun=1`) ride RTDE independently of External Control. The
   driver log shows `Setting digital output '16'` with no URP running. The
   URP is only needed for **arm motion**.

4. **A latched digital output causes a RECURRING fault at init.** `set_io`
   states are **latched** in the controller. If a tool DO is left ON
   (e.g. our cycle ended with pin 17 ON), every power-up re-drives it and
   re-trips the overcurrent — you get stuck unable to initialize.
   **Recovery:** load the **OnRobot installation** (URCap reclaims the tool
   I/O and resets the outputs) and re-init; or set **Tool Output Voltage →
   Off** before init. A full robot restart also resets to the OnRobot
   default installation.

5. **24 V can be set from software** via `set_io fun=4 state=24` and **read
   back** via `tool_data.tool_output_voltage`. `is_enabled` ⇔ voltage > 23 V.
   Do NOT trust the pendant Tool Output Voltage field alone — confirm via
   the readback before driving any pin.

6. **Analog Inputs vs Communication Interface** (Installation → Tool I/O,
   left panel) are mutually exclusive on the two analog pins:
   - **Digital Teach-mode path** needs **Analog Inputs** (so `analog_input2`
     gives width).
   - **RS485/Modbus path** needs **Communication Interface** (1M / Even /
     One / RX 1.5 / TX 3.5 — matches the OnRobot URCap's own settings).

7. **Two installation files, switch per task.** Keep the **OnRobot default**
   installation (Controlled by OnRobot, device = RG6) for URCap work, and a
   separate **`ros`** installation for ROS. `Installation → Load` to switch.
   **A robot restart reverts to the OnRobot default** — re-load `ros` after
   any restart.

---

## Digital-control workflow — HANDOFF for the next attempt

We are **not** pursuing the digital path right now (RS485/Modbus is the
chosen path — see decisions.md). But if it's retried later, here is the
clean procedure and the key hypothesis.

### Leading hypothesis to test first

**Fully UNINSTALL the OnRobot URCap** (not just set Device = "No
connection"). The reference README's premise is *Teach mode WITHOUT the
URCap installed*. With the URCap present, the gripper stays in RS485 mode
and its MCU holds the digital lines → the overcurrent we saw. Uninstalling
the URCap may drop the gripper into Teach mode so the digital lines become
real logic inputs.

> Caveat: uninstalling the URCap will break the user's two other PolyScope
> classes that depend on it. Reinstalling is required afterwards. Decide if
> the digital path is worth that churn vs. just using RS485/Modbus.

### Procedure (only after URCap uninstalled)

1. Pendant Tool I/O: **Controlled by: User**, **Tool Output Voltage: 24 V**,
   left panel **Analog Inputs** (analog_in[2]/[3] = Voltage).
2. Digital Output mode: **unknown** — sinking tripped DO0, sourcing tripped
   DO1, but both were *with the URCap installed*. Once the gripper is truly
   in Teach mode the correct mode may finally work. Start with **Sourcing
   (PNP)** (OnRobot is 24 V PNP logic) but **abort on the first fault**.
3. Run the existing helper [tests/onrobot_io_grip.py](../tests/onrobot_io_grip.py)
   — it already does the correct sequence: `fun=4` 24 V → **read-back
   confirm >23 V** → 5 s boot wait → wake-up toggle (pin 16 HIGH→LOW) →
   then open/close, with feedback on `digital_in_states[16/17]`.
4. **SAFETY GATE:** if the wake-up toggle (or the first close) trips a
   tool-connector overcurrent, **STOP immediately**. It means the gripper
   is still not in Teach mode. Do not keep toggling — repeated trips stress
   the tool driver. Recover per gotcha #4 above.

### What is already done (no need to rebuild)

- [tests/onrobot_io_grip.py](../tests/onrobot_io_grip.py) is written with the
  correct power-on + wake-up sequence and the reference feedback mapping
  (DI16=state, DI17=ready). It **cannot** re-trip the *first* fault on its
  own because it confirms 24 V before driving pin 16 — but it can still trip
  if the gripper isn't in Teach mode (that's the open risk above).

---

## Current decision (2026-05-28)

**Digital path = parked.** The working ROS 2 gripper path is **RS485/Modbus
over the UR tool communication bridge** (`use_tool_communication:=true` +
a Modbus client implementing the OnRobot RG6 register map). It uses the
gripper's native channel — the same one the URCap uses — so there is **no
overcurrent risk** (RS485 is a comm bus, not a driven power line), and it
coexists with External Control for arm motion. Reference:
[tonydle/ur_onrobot](https://github.com/tonydle/ur_onrobot) (params match
this cell's pendant exactly: 1M / Even / One / 24 V). See
[decisions.md](decisions.md) for the locked decision.

---

## Last updated

2026-05-28 (cell session — digital tool-I/O path tripped tool-connector
overcurrent twice; root-caused to gripper not in Teach mode; RS485/Modbus
chosen as the path forward).

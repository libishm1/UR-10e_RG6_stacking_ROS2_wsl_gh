# Path B (robot_ws) vs ROS 2 driver — comparison

## Purpose

Two different ways to drive the same UR10e + RG6 cell live in two
different workspaces on this machine. This page explains how they're
similar, how they differ, and which to reach for in a given situation.

- **Path B** = the SFTP + Dashboard URScript-deploy workflow in
  `D:\robot_ws\robots\outputs\2026-05-10\path_b\urp_deploy.py`. Pure
  Python, ROS-free. Source of the convention:
  `D:\robot_ws\robots\wiki\ur10e_rg6\path_b_deploy.md`.
- **ROS 2 driver path** = what THIS workspace (`~/ur_rg6_ws`) uses:
  `ur_robot_driver` + MoveIt + the External Control URCap. Continuous
  streaming over a reverse channel.

## Concrete artifact comparison

The user's verified `dodectest3.urp` (`D:\robot_ws\reference\dodectest3.urp`)
is a real Path B template. Decompressing it shows what a Path B program
looks like inside:

```xml
<URProgram name="dodectest3" installation="default" ...
           robotSerialNumber="20255201551" crcValue="3618013069">
  <kinematics status="LINEARIZED" validChecksum="true">
    <deltaTheta value="-1.137e-7, -1.408, 1.372, 0.036, -9.45e-8, 9.36e-8"/>
    <a value="4.12e-5, -0.0993, -0.5711, 3.00e-5, 2.27e-5, 0.0"/>
    <d value="0.1808, 429.38, -433.06, 3.858, 0.1198, 0.1156"/>
    <alpha value="1.5702, -0.00141, 0.00553, 1.5704, -1.5701, 0.0"/>
    ...
  </kinematics>
  <children>
    <MainProgram runOnlyOnce="true">
      <children>
        <Contributed strategyClass="com.onrobot.urcap.unified.OR_RG"
                     strategyProgramNodeType="RG Grip"
                     strategyURCapDeveloper="OnRobot A/S">
          <dataModel>
            <data key="rg-target-force" value="80.0"/>
            <data key="rg-target-width" value="0.15"/>
          </dataModel>
        </Contributed>
        <Script type="File">
          <cachedContents>
            global HOME_q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]
            movej(HOME_q, a=3.1416, v=0.7854, r=0.01)
            movel(WP_1, a=1.0, v=0.125, r=0.01)
            ...
            rg_grip(113.0, 40.0, tool_index=0, blocking=True, depth_comp=False, popupmsg=True)
            rg_payload_set(mass=0.0, tool_index=0, use_guard=True)
            ...
          </cachedContents>
          <file>/programs/usbdisk/classtrials/1LanaxHanoof.script</file>
        </Script>
      </children>
    </MainProgram>
  </children>
</URProgram>
```

Two things to notice:
1. `HOME_q = [1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]` is
   the **same vector** we use in this workspace's SRDF home,
   `initial_positions.yaml`, `play_pickplace.py` HOME_Q, and
   `real_hw_smoke.py`. Same physical pose on the same cell.
2. The `<kinematics>` block is the cell's **calibrated DH parameters**
   — the same numbers `ros2 launch ur_calibration calibration_correction.launch.py`
   extracts as a yaml. Path B bakes them into the program; ROS 2 reads
   them at launch.

## Side-by-side: how a "pick this up" command actually executes

| Step                            | Path B                                                                                    | ROS 2 driver                                                                                  |
|---------------------------------|-------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| Author motion                   | Generate URScript text (C# `UrCapFormatter` or Python)                                    | Build `MotionPlanRequest` in Python / C++; MoveIt plans                                       |
| Transport to cabinet            | SFTP `.urp`+`.script` to `/programs/`                                                     | Cabinet **dials back** to WSL on 50001/50002 (after Play); driver streams trajectory points  |
| Start                           | Dashboard `load <name>.urp` then `play` (TCP 29999)                                       | One-time Play on External Control program; thereafter streaming is automatic                  |
| Gripper                         | `rg_grip(...)` inline in the URScript that's running                                       | `std_msgs/String` to `/urscript_interface/script_command`; goes to port 30002 as secondary program |
| Feedback during execution       | None (Dashboard polls `programState` from outside)                                         | RTDE 500 Hz state + `/joint_states` + `/scaled_joint_trajectory_controller` status            |
| Stop                            | Dashboard `stop`                                                                          | MoveIt cancel goal, or kill the trajectory controller                                         |
| Cancel/replan mid-motion        | Re-deploy a new .urp                                                                      | Send a new MoveGroup goal; driver pre-empts                                                   |
| Cleanup                         | SFTP `rm -f /programs/<name>.{urp,script}`                                                | Nothing — driver state lives in RAM only                                                      |

## Network model — the most important difference

| Aspect                       | Path B                                | ROS 2 driver                                                                |
|------------------------------|----------------------------------------|-----------------------------------------------------------------------------|
| Connections initiated by laptop | TCP 22 (SFTP) + TCP 29999 (Dashboard) | TCP 30002 (URScript) + TCP 30004 (RTDE) + TCP 29999 (Dashboard ops)         |
| Connections initiated by cabinet | NONE                                | **TCP 50001 + 50002 back to laptop** (the WSL2 hard part)                   |
| Implication for WSL2         | Just works in any mode (NAT, mirrored, bridged) — outbound only | Needs **mirrored** OR **bridged** OR **netsh portproxy** so cabinet can reach the listener |
| Firewall on laptop           | Only outbound — usually open by default | Must allow inbound TCP on 50001-50002                                       |

**This is why Path B was the first thing to work in `D:\robot_ws`.**
The reverse-channel requirement of the ROS 2 driver path is the
single biggest gotcha on WSL2. Path B sidesteps it entirely.

## URCap requirements on the pendant

| URCap                          | Path B                                                      | ROS 2 driver                                                  |
|--------------------------------|-------------------------------------------------------------|---------------------------------------------------------------|
| External Control (UR)          | **not used**                                                 | **required** (accepts driver's reverse connection)            |
| OnRobot (RG6)                  | **required** for `rg_grip()` (inline in URScript)            | **required** for `rg_grip()` (via `/urscript_interface/...`)  |

Both paths need the OnRobot URCap because both invoke `rg_grip()`. The
URCap is what knows how to drive the gripper through tool I/O —
neither path can replace it. The difference is *who calls* `rg_grip`:
PolyScope running a `.urp` (Path B) vs an external client publishing
the URScript line (ROS 2).

## When to reach for which

**Use Path B when:**
- You have an operator-authored `.urp` to replay verbatim (the
  `dodectest3.urp` use case).
- You want a deterministic, "press play, walk away" deployment with
  no live ROS dependency.
- You're on a network where the cabinet can't dial back to your
  laptop (corporate firewall, VPN, no mirrored/bridged WSL).
- You need offline pre-validated trajectories (URScript was written
  and tested separately).

**Use the ROS 2 driver path when:**
- You want continuous MoveIt planning (Cartesian goals, collision-aware,
  re-planning).
- You want state feedback at 500 Hz for closed-loop control.
- You're integrating with the rest of a ROS 2 robotic stack
  (perception, behaviour trees, MoveIt Servo, etc.).
- You want RViz visualisation of the live robot.

**Hybrid?** Yes — they cohabit on the same cabinet. ROS 2 driver
streams arm trajectories live; for a particularly tricky gripper
sequence that needs URCap-only functions or full payload management,
you can deploy a brief `.urp` via Path B, run it, then re-attach the
ROS 2 driver. The OnRobot URCap is shared state on the pendant.

## Override gates (safety culture)

Path B has a hard 5-keyword override phrase requirement before any
deploy will run (encoded in `urp_deploy.py::assert_override`). The
ROS 2 driver has none — the safety contract is "press Play on the
pendant" + the velocity scaling defaults in scripts. For real-hardware
sessions in THIS workspace, that culture should propagate: lean on
`real_hw_smoke.py`'s `--yes` requirement and the `MAX_DELTA_RAD`
hard cap.

## Reference files

- `D:\robot_ws\robots\outputs\2026-05-10\path_b\urp_deploy.py` —
  canonical Path B implementation.
- `D:\robot_ws\robots\outputs\2026-05-10\path_b\urcap_grip_move_test.py`
  — Path B applied to a small gripper+move sequence.
- `D:\robot_ws\reference\dodectest3.urp` — template `.urp` referenced
  above; verified V3 on 2026-05-10.
- `D:\robot_ws\reference\01_Dashboard_GHScript.cs` — Dashboard client
  (port 29999) used inside Grasshopper. Same protocol the ROS 2
  driver uses for `programState` / `play` / `stop`.
- `D:\robot_ws\reference\03_URScript_Sender_GHScript.cs` — sends
  URScript over port 30002 from C#. This is the C# equivalent of
  `ros2 topic pub /urscript_interface/script_command`.
- `~/ur_rg6_ws/tests/real_hw_smoke.py` — ROS 2 path minimal test.
- `~/ur_rg6_ws/tests/play_pickplace.py` — ROS 2 path full sequence.

## Last updated

2026-05-24.

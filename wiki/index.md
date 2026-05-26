# UR10e + RG6 wiki

Curated, durable findings for this workspace. The pattern is borrowed from
`D:\robot_ws\robots\wiki\`: notes that outlive a single session live here;
session-state and "what we just did" go to [`SESSION_HANDOFF.md`](../SESSION_HANDOFF.md).

When you (Claude or human) do research, dig through code, or arrive at a
non-obvious finding — promote the durable part to a wiki page here. Do not
let it die in a chat transcript.

## Pages

- [Real-hardware connection (ROS 2 driver path)](real_hw_connection.md) — what
  the workspace uses today: `ur_robot_driver` + MoveIt + RViz. Network model,
  topics, gripper path, pre-flight, smoke test sequence.
- [Path B vs ROS 2 driver comparison](path_b_vs_ros_driver.md) — robot_ws's
  SFTP+Dashboard URScript-deploy workflow vs the ROS 2 driver's streaming
  External Control URCap path. When to use which.
- [RG6 control mechanisms](rg6_control_mechanisms.md) — the three ROS 2 paths
  shipped in the reference repos: (A) onrobot_interface C++ ros2_control plugin
  via UR tool I/O pins, (B) onrobot_driver Python standalone node (same pins),
  (C) URScript topic via the URCap. Which one to pick and why.
- [Locked decisions](decisions.md) — design choices that shouldn't drift
  across sessions. Currently: **RG6 control = Mechanism C** (URScript topic).
- [Launch files](launch_files.md) — inventory of every `.launch.py` in the
  workspace, their args, what they bring up, when to use which.
- [Real-hardware validation plan](real_hw_validation_plan.md) — 9-phase
  step-by-step checklist with pass/abort criteria, validation log
  template, and rollback procedure. Run end-to-end at the cell with
  a hand on the E-stop.
- [RViz GPU rendering under WSL2](rviz_gpu_rendering.md) — verified
  D3D12 passthrough via WSLg to Intel UHD 630. How to confirm,
  what to do if you ever see `llvmpipe` fallback.
- [RViz visual orientation mismatch (early attempts)](rviz_visual_orientation_mismatch.md) —
  full attempt log: calibration extraction, URDF base rotation, mesh
  rotation. **Superseded** by the shoulder-pan sign-flip finding.
- [Shoulder-pan sign mismatch (FIX FOUND)](shoulder_pan_sign_mismatch.md) —
  URDF and cabinet use opposite shoulder_pan_joint sign convention. Flip
  HOME_Q[0] to -pi/2 → RViz matches real cell. **Critical caveat for
  real hardware:** sending -pi/2 to real cabinet sends arm to the
  wrong side; needs care during Phase 5+ deployment.
- [Known bugs and workarounds (catalog)](known_bugs_and_workarounds.md) —
  living index of every "burnt by this" from the workspace. Future
  Claude sessions: search here first before re-discovering.
- [WSL2 networking deep-dive](../docs/WSL2_UR10e_NETWORKING.md) — fallback
  ladder (mirrored → bridged → NAT+portproxy → native Linux). Lives outside
  the wiki because it's a setup guide users follow step-by-step.

## Cross-references

- `D:\robot_ws\robots\wiki\ur10e_rg6\` — the original (C#/Mecha-side) wiki
  for the same cell. Source of truth for cell network config, calibration,
  Path B, RTDE. We reuse its values here.
- Persistent Claude memory at
  `C:\Users\libish m\.claude\projects\c--Users-libish-m\memory\`:
  `project_ur10e_rg6_workspace.md`, `reference_ur10e_cell_network.md`,
  `reference_path_b_deploy.md`.

## Conventions

- Each page starts with a one-line **Purpose**.
- **Confirmed facts** vs **Working assumptions** sections — never blur them.
- Cite sources inline (GitHub URLs, robot_ws paths, doc URLs).
- Date each "last updated" line at the bottom so stale pages stand out.

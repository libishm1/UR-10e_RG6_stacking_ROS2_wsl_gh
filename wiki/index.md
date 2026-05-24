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

# Locked decisions

## Purpose

Permanent record of design choices that shouldn't drift across sessions.
Each entry is a one-liner decision + the reasons that locked it. If a
decision is reversed, append a new entry — don't edit the old one.

Pattern borrowed from `D:\robot_ws\robots\wiki\project_management\decisions.md`.

---

## 2026-05-24 — RG6 real-hardware control: Mechanism C (URScript topic)

**Decision.** For real-hardware gripper control, use **Mechanism C**:
publish single-line `rg_grip(width_mm, force_N, ...)` URScript to
`/urscript_interface/script_command`. Do NOT use the
`onrobot_interface` C++ ros2_control plugin (Mechanism A) or the
`onrobot_driver` Python node (Mechanism B), even though both are
shipped in the reference repos.

**Why.**

1. **Continuous width and force.** A and B are effectively binary
   (open/close + low-force mode bit). The pick-and-place sequence
   needs to grip at 50 / 60 / 70 mm with controllable force; only
   C gives mm- and N-level control.
2. **No RG6 ROS boilerplate required.** C works with stock
   `ur_robot_driver` + any MoveIt-for-UR. A needs the URDF plugin
   block + `rg6_joint` + controller-manager config; B needs the
   `onrobot_driver` Python node running. C removes a whole layer of
   things that can break.
3. **URCap on pendant is the same OnRobot URCap already installed.**
   A and B require the URCap in "mounted-to-UR / pin mode" — we'd
   have to reconfigure the pendant. C uses the URCap in its default
   "URScript-callable" mode, which the cell is already set up for
   (verified 2026-05-10 on dodectest3.urp).
4. **Same code path as our verified sim runs.** `play_pickplace.py`
   already uses `/urscript_interface/script_command` for the
   gripper in `--real-gripper` mode; switching to A/B would mean a
   different code path on real hardware than the one verified in
   sim.

**Implications.**

- The `<plugin>onrobot_interface::OnRobotHardwareInterface</plugin>` block
  in [`src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro`](../src/Universal_Robots_ROS2_Description/urdf/ur10e_rg6.urdf.xacro)
  is now DORMANT for real-hardware. It's still selected when
  `use_fake_hardware:=false`, but we'll bypass it in practice by
  using mechanism C from our scripts. Keep the URDF wiring for now
  (no need to rip it out) — leave as future cleanup if we ever
  confirm we never want A.
- The `rg6_gripper_controller` (joint_trajectory_controller) is
  still useful for SIM runs (`play_pickplace.py` default mode) so
  ghost-robot RViz shows the gripper opening/closing. Keep it.
- Width-mm ↔ angle-rad cubic in
  [`config/rg6_width_calibration.yaml`](../src/ur10e_rg6_moveit_config/config/rg6_width_calibration.yaml)
  is only used by the SIM path now — it's irrelevant to real
  hardware. Keep for sim parity.

**What changes in practice.** Nothing in the scripts —
`play_pickplace.py --real-gripper` and
`real_hw_smoke.py --yes --real-gripper` are already on path C. This
decision just locks the choice and says "don't get tempted into A or
B without a real reason".

**Reference.** [`rg6_control_mechanisms.md`](rg6_control_mechanisms.md)
for the full A/B/C comparison and the code citations.

---

## Last updated

2026-05-24.

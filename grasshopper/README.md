# Grasshopper bridge for UR10e + RG6

Two files in here:
- **`ur10e_rg6_gh.py`** — the unified Python 3 runtime. Drop this into one GHPython
  component set to **Python 3 (CPython)** mode in Rhino 8.
- **`ur10e_rg6.ghx`** — a best-effort pre-wired Grasshopper file. If it refuses to
  open (the Grasshopper file format is fussy and I can't verify it from here),
  use `ur10e_rg6_gh.py` as the guaranteed-working fallback.

## Prereqs

1. **Rhino 8** with Python 3 enabled (Rhino 7 won't work — IronPython 2 can't do
   modern WebSockets).
2. The Docker stack from `../docker/` running with at least these two services up:
   - `ur_fake` (or `ur_real`) — provides the actual ROS 2 graph
   - `rosbridge` — exposes `ws://localhost:9090`

## Manual setup (the reliable path)

1. New Grasshopper definition. Drop one **GHPython** component.
2. Right-click the component header → `Python 3 (CPython)` (this is essential).
3. Double-click to open the editor. Paste the contents of `ur10e_rg6_gh.py`.
4. Add the following named inputs on the left side (right-click → `Set Input
   Parameter` or use the input zoom-in `+`):

   | Name         | Type      | Access | Default value (via panel/slider/toggle)        |
   |--------------|-----------|--------|------------------------------------------------|
   | `host`       | str       | item   | panel: `localhost`                             |
   | `port`       | int       | item   | panel: `9090`                                  |
   | `connect`    | bool      | item   | Boolean Toggle                                 |
   | `mode`       | str       | item   | panel: `direct` or `moveit`                    |
   | `target`     | float     | list   | 6 sliders or a `Merge` of 6 values             |
   | `duration`   | float     | item   | slider, e.g. 4.0 (used in `direct` mode only)  |
   | `vel_scale`  | float     | item   | slider 0..1 (used in `moveit` mode only)       |
   | `move`       | bool      | item   | Button (momentary)                             |
   | `gripper`    | bool      | item   | Boolean Toggle (True = close)                  |
   | `trigger_gr` | bool      | item   | Button (momentary)                             |
   | `tick`       | generic   | item   | wire a `Timer` (100 ms) here                   |

   **Mode selection:**
   - `direct` — raw `FollowJointTrajectory` action on
     `/scaled_joint_trajectory_controller`. Fast and predictable, **no
     collision checking**. The Docker `ur_fake` or `ur_real` service is
     enough.
   - `moveit` — sends a `MoveGroup` action goal so MoveIt 2 plans a
     collision-free path and executes it. Requires the `moveit` Docker
     service running side-by-side with the driver.

5. Set these outputs on the right side: `ok`, `names`, `positions`, `tcp_pos`,
   `tcp_quat`, `log`.
6. Right-click each output → `Tree Access` left as default `Item`. Attach a
   `Panel` to `log` and `ok` for quick visibility.
7. Attach a **Timer** (from `Params/Util`) to the `tick` input. Set its interval
   to 100 ms. This is what keeps your subscribed messages flowing through the
   GH solver.

## Quick verification flow

1. Flip `connect` to True. The `ok` panel should print `True` within a second.
2. Watch `positions` change as you move sliders inside the UR teach pendant (or
   it just shows constant fake-hardware values when `ur_fake` is up).
3. Set `target` to `[0, -1.57, 1.57, -1.57, -1.57, 0]`, `duration=4.0`.
4. Push the `move` button. In RViz the arm should glide to that pose.
5. Push the `trigger_gr` button — check the UR pendant I/O panel: digital out 0
   should flip. Wire the RG6 actuation through that pin per your wiring.

## Notes

- `tcp_pos` is reported **in millimetres** (Rhino default unit). The internal
  ROS frames are in metres; the conversion lives at the bottom of the
  `_resolve_pose()` block.
- The `target` array is in **radians** and must list values in this order:
  `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`.
- For real-hardware safety, set `duration` long enough that the average joint
  speed is well below your UR speed slider's limit. Otherwise the controller
  scales the trajectory or rejects it.

## How to map to MoveIt 2 instead of the raw trajectory controller

Once the MoveIt config package (`src/ur10e_rg6_moveit_config`) is launched,
swap the action target in the script from
`/scaled_joint_trajectory_controller/follow_joint_trajectory` to MoveIt's
`/move_action`. You also need to send a `moveit_msgs/action/MoveGroup` goal
with constraints, which is meaningfully more complex — easier to call
MoveIt's `/compute_cartesian_path` service from Grasshopper and feed the
resulting joint trajectory into the existing controller path. Happy to wire
that up next if you want.

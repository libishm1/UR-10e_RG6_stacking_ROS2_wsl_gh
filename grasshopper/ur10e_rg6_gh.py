# ur10e_rg6_gh.py
# Drop into a Rhino 8 GHPython component set to Python 3 (CPython) mode.
# The script directive below auto-pip-installs roslibpy.
#
# Component INPUTS (right-click each input dot to rename to these):
#   host       (str, default "localhost")
#   port       (int, default 9090)
#   connect    (bool toggle)        master on/off for the WebSocket
#   mode       (str)                "direct" or "moveit"  -- chooses goal sink
#   target     (list[float] len 6)  desired UR joint goal in radians
#   duration   (float)              seconds for the trajectory (direct mode)
#   vel_scale  (float, 0..1)        MoveIt velocity scaling factor
#   move       (bool button)        fires once: sends the goal
#   gripper    (bool toggle)        True = close RG6, False = open
#   trigger_gr (bool button)        fires once: sends the gripper command
#   tick       (anything timed)     wire a Timer to this to keep IO flowing
#
# Component OUTPUTS:
#   ok         (bool)               WS connected?
#   names      (list[str])          live joint names from /joint_states
#   positions  (list[float])        live joint positions
#   tcp_pos    (Point3d)            live world-frame RG6 TCP position
#   tcp_quat   (list[float])        live world-frame RG6 TCP quaternion (x,y,z,w)
#   log        (str)                last status message

# r: roslibpy

import math
import time
import roslibpy
import roslibpy.actionlib as actionlib
import Rhino.Geometry as rg

# ------------------------------------------------------------------
# Per-component cache (persisted between solutions on this instance)
# ------------------------------------------------------------------
S = ghenv.Component
if not hasattr(S, "_rg6_state"):
    S._rg6_state = {
        "ros": None,
        "joint_msg": {"name": [], "position": []},
        "tf_world_tcp": None,
        "log": "",
        "move_armed": False,
        "grip_armed": False,
    }
state = S._rg6_state

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
RG6_TIP_FRAME = "rg6_tcp"
BASE_FRAME = "world"

# MoveIt 2 endpoints used in "moveit" mode (must match ur10e_rg6_moveit_config)
MOVEIT_GROUP = "ur_manipulator"
MOVEIT_ACTION = "/move_action"
MOVEIT_ACTION_TYPE = "moveit_msgs/action/MoveGroup"

# RG6 gripper: single master joint rg6_joint, range 0 (closed) → 1.3 (full open).
# Default to ~10 mm gap (0.08 rad) — safe boot position the URDF uses too.
# The gripper controller is a JointTrajectoryController; commands are sent as
# trajectory_msgs/JointTrajectory on /rg6_gripper_controller/joint_trajectory.
RG6_JOINT = "rg6_joint"
RG6_TRAJ_TOPIC = "/rg6_gripper_controller/joint_trajectory"
# RG6 joint convention (calibrated, see config/rg6_width_calibration.yaml):
#   rg6_joint = 0.00 rad → full open (~153 mm gap)
#   rg6_joint = 0.77 rad → ~70 mm gap (safe boot)
#   rg6_joint = 1.25 rad → ~1  mm gap (closed)
# Approx cubic angle(width_mm): ((-2.61e-7 * w + 4.18e-5) * w − 0.00862) * w + 1.295
RG6_OPEN_RAD   = 0.05    # ≈ 150 mm full open
RG6_SAFE_RAD   = 0.77    # ≈  70 mm safe
RG6_CLOSED_RAD = 1.25    # ≈   1 mm closed
RG6_MOVE_TIME = 1.5    # seconds per gripper command — safe default


def _log(msg):
    state["log"] = str(msg)


# ------------------------------------------------------------------
# Connect / disconnect
# ------------------------------------------------------------------
ros = state["ros"]

if connect:
    if ros is None or not ros.is_connected:
        try:
            ros = roslibpy.Ros(host=host or "localhost", port=int(port or 9090))
            ros.run()
            state["ros"] = ros
            _log("connected")

            # /joint_states subscriber (sensor_msgs/msg/JointState)
            js_topic = roslibpy.Topic(ros, "/joint_states", "sensor_msgs/msg/JointState")
            js_topic.subscribe(lambda m: state.__setitem__("joint_msg", m))

            # /tf subscriber — accumulate transforms keyed by (parent, child)
            state.setdefault("tf_table", {})

            def _on_tf(msg):
                for tr in msg.get("transforms", []):
                    key = (tr["header"]["frame_id"], tr["child_frame_id"])
                    state["tf_table"][key] = tr

            tf_topic = roslibpy.Topic(ros, "/tf", "tf2_msgs/msg/TFMessage")
            tf_topic.subscribe(_on_tf)
            tf_static = roslibpy.Topic(ros, "/tf_static", "tf2_msgs/msg/TFMessage")
            tf_static.subscribe(_on_tf)
        except Exception as e:
            _log("connect failed: %s" % e)
            state["ros"] = None
            ros = None
else:
    if ros is not None:
        try:
            ros.terminate()
        except Exception:
            pass
        state["ros"] = None
        ros = None
        _log("disconnected")

ok = bool(ros and ros.is_connected)

# ------------------------------------------------------------------
# Live joint state output
# ------------------------------------------------------------------
jm = state["joint_msg"]
names = list(jm.get("name", []))
positions = list(jm.get("position", []))

# ------------------------------------------------------------------
# Live RG6 TCP from /tf (chase the chain world -> rg6_tcp)
# ------------------------------------------------------------------
tcp_pos = None
tcp_quat = []
tf_table = state.get("tf_table", {})


def _resolve_pose(target_frame, source_frame=BASE_FRAME, depth=12):
    """Compose transforms along the parent chain. Returns (xyz, quat) or None."""
    # Walk from target_frame back toward source_frame.
    chain = []
    cur = target_frame
    for _ in range(depth):
        parent = None
        for (p, c), tr in tf_table.items():
            if c == cur:
                parent = p
                chain.append(tr)
                break
        if parent is None:
            break
        if parent == source_frame:
            chain.append(None)  # sentinel
            break
        cur = parent
    if not chain or chain[-1] is not None:
        return None
    chain = chain[:-1]  # drop sentinel

    # Compose: chain[0] is leaf, chain[-1] is closest to source.
    px, py, pz = 0.0, 0.0, 0.0
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    for tr in reversed(chain):
        t = tr["transform"]["translation"]
        r = tr["transform"]["rotation"]
        # quat-rotate (t.x,t.y,t.z) by (qx,qy,qz,qw) and add to (px,py,pz)
        tx, ty, tz = t["x"], t["y"], t["z"]
        # rotate vec by current quat
        rx = (1 - 2 * qy * qy - 2 * qz * qz) * tx + (2 * qx * qy - 2 * qz * qw) * ty + (2 * qx * qz + 2 * qy * qw) * tz
        ry = (2 * qx * qy + 2 * qz * qw) * tx + (1 - 2 * qx * qx - 2 * qz * qz) * ty + (2 * qy * qz - 2 * qx * qw) * tz
        rz = (2 * qx * qz - 2 * qy * qw) * tx + (2 * qy * qz + 2 * qx * qw) * ty + (1 - 2 * qx * qx - 2 * qy * qy) * tz
        px += rx; py += ry; pz += rz
        # multiply quats: q = q * r
        nx = qw * r["x"] + qx * r["w"] + qy * r["z"] - qz * r["y"]
        ny = qw * r["y"] - qx * r["z"] + qy * r["w"] + qz * r["x"]
        nz = qw * r["z"] + qx * r["y"] - qy * r["x"] + qz * r["w"]
        nw = qw * r["w"] - qx * r["x"] - qy * r["y"] - qz * r["z"]
        qx, qy, qz, qw = nx, ny, nz, nw
    return (px, py, pz), (qx, qy, qz, qw)


pose = _resolve_pose(RG6_TIP_FRAME)
if pose is not None:
    (px, py, pz), q = pose
    tcp_pos = rg.Point3d(px * 1000.0, py * 1000.0, pz * 1000.0)  # m -> mm for Rhino default
    tcp_quat = list(q)

# ------------------------------------------------------------------
# Move arm — edge-trigger on `move`
# Two modes:
#   "direct"  → raw FollowJointTrajectory action on the UR controller.
#               Fast and predictable, but does NO collision checking.
#   "moveit"  → MoveIt 2 /move_action: plans a collision-free path, then
#               executes through the same controller. Requires the MoveIt
#               demo or move_group launch to be running.
# ------------------------------------------------------------------
_mode = (mode or "direct").strip().lower()

def _send_direct(t, dur):
    dur_sec = int(dur)
    dur_nsec = int((dur - dur_sec) * 1e9)
    client = actionlib.ActionClient(
        ros,
        "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        "control_msgs/action/FollowJointTrajectory",
    )
    goal = actionlib.Goal(client, roslibpy.Message({
        "trajectory": {
            "joint_names": UR_JOINTS,
            "points": [{
                "positions": t,
                "velocities": [0.0] * 6,
                "time_from_start": {"sec": dur_sec, "nanosec": dur_nsec},
            }],
        }
    }))
    goal.on("result", lambda r: _log("direct: done"))
    goal.send()
    return goal


def _send_moveit(t, vscale):
    """Build a MoveGroup goal with joint constraints; plan + execute."""
    joint_constraints = [{
        "joint_name": jname,
        "position": float(jpos),
        "tolerance_above": 0.01,
        "tolerance_below": 0.01,
        "weight": 1.0,
    } for jname, jpos in zip(UR_JOINTS, t)]

    motion_plan_request = {
        "workspace_parameters": {
            "header": {"frame_id": BASE_FRAME, "stamp": {"sec": 0, "nanosec": 0}},
            "min_corner": {"x": -2.0, "y": -2.0, "z": -2.0},
            "max_corner": {"x":  2.0, "y":  2.0, "z":  2.0},
        },
        "start_state": {"is_diff": True},  # use current state
        "goal_constraints": [{
            "name": "gh_goal",
            "joint_constraints": joint_constraints,
            "position_constraints": [],
            "orientation_constraints": [],
            "visibility_constraints": [],
        }],
        "path_constraints": {"name": "", "joint_constraints": [],
                             "position_constraints": [], "orientation_constraints": [],
                             "visibility_constraints": []},
        "trajectory_constraints": {"constraints": []},
        "reference_trajectories": [],
        "pipeline_id": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "group_name": MOVEIT_GROUP,
        "num_planning_attempts": 10,
        "allowed_planning_time": 5.0,
        "max_velocity_scaling_factor": float(vscale),
        "max_acceleration_scaling_factor": float(vscale),
        "cartesian_speed_limited_link": "",
        "max_cartesian_speed": 0.0,
    }
    planning_options = {
        "planning_scene_diff": {"is_diff": True, "robot_state": {"is_diff": True}},
        "plan_only": False,
        "look_around": False,
        "look_around_attempts": 0,
        "max_safe_execution_cost": 0.0,
        "replan": False,
        "replan_attempts": 0,
        "replan_delay": 0.0,
    }

    client = actionlib.ActionClient(ros, MOVEIT_ACTION, MOVEIT_ACTION_TYPE)
    goal = actionlib.Goal(client, roslibpy.Message({
        "request": motion_plan_request,
        "planning_options": planning_options,
    }))

    def _on_result(res):
        code = res.get("error_code", {}).get("val", 0)
        _log("moveit: done (error_code=%s)" % code)

    goal.on("result", _on_result)
    goal.send()
    return goal


if ok and move and not state["move_armed"]:
    state["move_armed"] = True
    try:
        if not target or len(list(target)) != 6:
            _log("move: target must be 6 floats (radians)")
        else:
            t = [float(x) for x in target]
            if _mode == "moveit":
                vs = float(vel_scale) if vel_scale is not None else 0.2
                vs = max(0.01, min(1.0, vs))
                _send_moveit(t, vs)
                _log("moveit: sent group=%s vscale=%.2f" % (MOVEIT_GROUP, vs))
            else:
                dur = max(0.5, float(duration or 4.0))
                _send_direct(t, dur)
                _log("direct: sent %s rad over %.1fs" % (t, dur))
    except Exception as e:
        _log("move error: %s" % e)
elif not move:
    state["move_armed"] = False

# ------------------------------------------------------------------
# Gripper — edge-trigger on `trigger_gr`
# Drives the rg6_gripper_controller via Float64MultiArray.
# gripper=True  -> close (0.0 rad)
# gripper=False -> open  (1.3 rad)
# A repeat-publish keeps the controller's last command latched.
# ------------------------------------------------------------------
if ok and trigger_gr and not state["grip_armed"]:
    state["grip_armed"] = True
    try:
        topic = roslibpy.Topic(ros, RG6_TRAJ_TOPIC,
                               "trajectory_msgs/msg/JointTrajectory")
        topic.advertise()
        target = RG6_CLOSED_RAD if gripper else RG6_OPEN_RAD
        sec = int(RG6_MOVE_TIME)
        nsec = int((RG6_MOVE_TIME - sec) * 1e9)
        msg = roslibpy.Message({
            "joint_names": [RG6_JOINT],
            "points": [{
                "positions": [float(target)],
                "velocities": [0.0],
                "time_from_start": {"sec": sec, "nanosec": nsec},
            }],
        })
        topic.publish(msg)
        topic.unadvertise()
        _log("rg6: %s (%.2f rad over %.1fs)" %
             ("close" if gripper else "open", target, RG6_MOVE_TIME))
    except Exception as e:
        _log("rg6 call error: %s" % e)
elif not trigger_gr:
    state["grip_armed"] = False

log = state["log"]

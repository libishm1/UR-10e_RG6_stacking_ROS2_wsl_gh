"""Just the move_group node + RViz, assuming someone else is publishing
/robot_description and running the ros2_control hardware interface.

Use this when you already have ur_control.launch.py running."""
import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _load(p): return open(p).read()


def generate_launch_description():
    ur_desc = FindPackageShare("ur_description")
    pkg_share = get_package_share_directory("ur10e_rg6_moveit_config")
    xacro_file = PathJoinSubstitution([ur_desc, "urdf", "ur10e_rg6.urdf.xacro"])

    use_fake = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")

    robot_description = {"robot_description": ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_fake_hardware:=", use_fake,
            " robot_ip:=", robot_ip,
        ]),
        value_type=str,
    )}
    srdf = {"robot_description_semantic": _load(os.path.join(pkg_share, "config", "ur10e_rg6.srdf"))}
    with open(os.path.join(pkg_share, "config", "kinematics.yaml")) as f:
        kinematics = {"robot_description_kinematics": yaml.safe_load(f)}
    # joint_limits.yaml + pilz_cartesian_limits.yaml BOTH live under
    # robot_description_planning. Loading them as separate dicts caused the
    # second to overwrite the first (moveit2 issue #1691) — Pilz then couldn't
    # find rg6_joint's velocity/acceleration limits and threw map::at.
    # We merge them in Python so both make it through.
    with open(os.path.join(pkg_share, "config", "joint_limits.yaml")) as f:
        _jl_raw = yaml.safe_load(f)
    with open(os.path.join(pkg_share, "config", "pilz_cartesian_limits.yaml")) as f:
        _pcart_raw = yaml.safe_load(f)
    robot_description_planning = {"robot_description_planning": {**_jl_raw, **_pcart_raw}}

    with open(os.path.join(pkg_share, "config", "ompl_planning.yaml")) as f:
        ompl_pipeline = {"ompl": yaml.safe_load(f)}
    with open(os.path.join(pkg_share, "config", "pilz_planning.yaml")) as f:
        pilz_pipeline = {"pilz_industrial_motion_planner": yaml.safe_load(f)}

    with open(os.path.join(pkg_share, "config", "moveit_controllers.yaml")) as f:
        moveit_controllers = yaml.safe_load(f)

    planning_pipelines = {
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "ompl",
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            srdf,
            kinematics,
            robot_description_planning,
            ompl_pipeline,
            pilz_pipeline,
            planning_pipelines,
            moveit_controllers,
            {"publish_robot_description_semantic": True,
             "allow_trajectory_execution": True,
             "moveit_manage_controllers": False},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1"),
        move_group,
    ])

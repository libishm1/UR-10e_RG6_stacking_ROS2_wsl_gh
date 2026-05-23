"""Standalone RViz with the MoveIt MotionPlanning panel."""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _load(p): return open(p).read()


def generate_launch_description():
    pkg_share = get_package_share_directory("ur10e_rg6_moveit_config")
    ur_desc = FindPackageShare("ur_description")
    xacro_file = PathJoinSubstitution([ur_desc, "urdf", "ur10e_rg6.urdf.xacro"])

    use_fake = LaunchConfiguration("use_fake_hardware")
    robot_description = {"robot_description": ParameterValue(
        Command(["xacro ", xacro_file, " use_fake_hardware:=", use_fake]),
        value_type=str,
    )}
    srdf = {"robot_description_semantic": _load(os.path.join(pkg_share, "config", "ur10e_rg6.srdf"))}
    with open(os.path.join(pkg_share, "config", "kinematics.yaml")) as f:
        kinematics = {"robot_description_kinematics": yaml.safe_load(f)}

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_share, "config", "moveit.rviz")],
        parameters=[robot_description, srdf, kinematics],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        rviz,
    ])

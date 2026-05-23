"""
End-to-end MoveIt 2 demo launch for the UR10e + RG6 stack.

Starts:
  * robot_state_publisher (URDF from the combined xacro)
  * ros2_control_node with controllers (fake hardware)
  * move_group
  * RViz with MotionPlanning panel

For real hardware run `ur_robot_driver/ur_control.launch.py` separately and
only launch `move_group.launch.py` + `moveit_rviz.launch.py` from this package.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _load(path):
    with open(path, "r") as f:
        return f.read()


def generate_launch_description():
    ur_desc = FindPackageShare("ur_description")
    moveit_cfg = FindPackageShare("ur10e_rg6_moveit_config")

    xacro_file = PathJoinSubstitution([ur_desc, "urdf", "ur10e_rg6.urdf.xacro"])
    srdf_file = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "ur10e_rg6.srdf",
    )
    kinematics_yaml = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "kinematics.yaml",
    )
    joint_limits_yaml = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "joint_limits.yaml",
    )
    ompl_yaml = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "ompl_planning.yaml",
    )
    moveit_controllers_yaml = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "moveit_controllers.yaml",
    )
    rviz_cfg = os.path.join(
        get_package_share_directory("ur10e_rg6_moveit_config"),
        "config", "moveit.rviz",
    )

    use_fake = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")

    robot_description_content = Command([
        "xacro ", xacro_file,
        " use_fake_hardware:=", use_fake,
        " robot_ip:=", robot_ip,
    ])

    robot_description = {"robot_description": robot_description_content}
    srdf = {"robot_description_semantic": _load(srdf_file)}

    import yaml
    with open(kinematics_yaml) as f:
        kinematics = {"robot_description_kinematics": yaml.safe_load(f)}
    with open(joint_limits_yaml) as f:
        joint_limits = {"robot_description_planning": yaml.safe_load(f)}
    with open(ompl_yaml) as f:
        ompl = yaml.safe_load(f)
    with open(moveit_controllers_yaml) as f:
        moveit_controllers = yaml.safe_load(f)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            srdf,
            kinematics,
            joint_limits,
            ompl,
            moveit_controllers,
            {"publish_robot_description_semantic": True,
             "allow_trajectory_execution": True,
             "default_planning_pipeline": "ompl",
             "planning_pipelines": ["ompl"]},
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            robot_description,
            srdf,
            kinematics,
            joint_limits,
            ompl,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("robot_ip",          default_value="127.0.0.1"),
        rsp,
        move_group,
        rviz,
    ])

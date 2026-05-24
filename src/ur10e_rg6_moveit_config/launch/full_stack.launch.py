"""End-to-end launch for the UR10e + RG6 + MoveIt stack on fake hardware.

Includes:
  1. ur_control.launch.py via the onrobot wrapper (UR controllers + RSP + RViz off)
  2. rg6_gripper_controller (JointTrajectoryController) spawn
  3. move_group with OMPL + Pilz pipelines
  4. MoveIt RViz

Use this instead of the 3 separate launches when you just want to bring
everything up.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_fake = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    pkg_share = get_package_share_directory("ur10e_rg6_moveit_config")

    # 1. UR control stack (controllers, RSP, etc.)
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("onrobot_description"), "launch",
                "ur10e_rg6_control.launch.py",
            ])
        ),
        launch_arguments={
            "use_fake_hardware": use_fake,
            "robot_ip": robot_ip,
        }.items(),
    )

    # 2. Gripper controller spawner (waits for controller_manager via the
    #    spawner's built-in service polling)
    rg6_jtc_yaml = os.path.join(pkg_share, "config", "rg6_jtc.yaml")
    spawn_gripper = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["rg6_gripper_controller",
                           "-t", "joint_trajectory_controller/JointTrajectoryController",
                           "-p", rg6_jtc_yaml,
                           "--controller-manager", "/controller_manager",
                           "--controller-manager-timeout", "30"],
                output="screen",
            ),
        ],
    )

    # 3. move_group with OMPL + Pilz (delay so controllers are up first)
    move_group = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "move_group.launch.py")),
                launch_arguments={"use_fake_hardware": use_fake}.items(),
            ),
        ],
    )

    # 4. RViz (delay so move_group is publishing semantic description)
    rviz = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "moveit_rviz.launch.py")),
                launch_arguments={"use_fake_hardware": use_fake}.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="true=mock_components; false=ur_robot_driver against real UR10e"),
        DeclareLaunchArgument("robot_ip", default_value="127.0.0.1",
                              description="IP of the real UR10e (ignored when use_fake_hardware:=true)"),
        ur_launch,
        spawn_gripper,
        move_group,
        rviz,
    ])

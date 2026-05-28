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
    # Tool RS485 (OnRobot RG6 over Modbus). Defaults OFF — unchanged for sim
    # and digital-path launches; enabled by scripts/launch_real_rs485.sh.
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_voltage = LaunchConfiguration("tool_voltage")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
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
            "use_tool_communication": use_tool_communication,
            "tool_voltage": tool_voltage,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars,
            "tool_tx_idle_chars": tool_tx_idle_chars,
            "tool_device_name": tool_device_name,
        }.items(),
    )

    # 1b. Static world -> base_link TF. The SRDF defines a `world_to_base`
    #     virtual joint, but that is a MoveIt PLANNING construct and is NOT
    #     published to TF (robot_state_publisher only knows the URDF, rooted
    #     at base_link). Without this, RViz's Fixed Frame "world" is
    #     unresolvable and the robot never renders. See
    #     wiki/known_bugs_and_workarounds.md.
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_static_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "base_link"],
        output="screen",
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
        DeclareLaunchArgument("use_tool_communication", default_value="false",
                              description="true=bridge tool RS485 for OnRobot RG6 Modbus (see launch_real_rs485.sh)"),
        DeclareLaunchArgument("tool_voltage", default_value="0"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        ur_launch,
        world_to_base_tf,
        spawn_gripper,
        move_group,
        rviz,
    ])

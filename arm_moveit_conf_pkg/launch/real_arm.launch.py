#!/usr/bin/env python3
"""
real_arm.launch.py
==================
Launches the arm with the real ArmHardwareInterface.

Supports two transports — choose at launch time:

  # UART Serial (default):
  ros2 launch arm_moveit_conf_pkg real_arm.launch.py

  # WiFi TCP:
  ros2 launch arm_moveit_conf_pkg real_arm.launch.py transport:=wifi esp_ip:=***.***.*.**

The launch arg is injected into the URDF's <param name="transport"> tag via
xacro-style string substitution so the hardware plugin sees the correct value
at runtime.  The URDF itself still has transport=serial as the static default
(so the URDF is valid on its own), but the launch argument always wins.

The Launching Sequence:
immediately:   robot_state_publisher
               ros2_control_node (loads hardware plugin)
                    │
after 3s:      joint_state_broadcaster_spawner  →  publishes /joint_states
                    │ (on exit)
               arm_controller  →  receives trajectories from MoveIt
                    
after 5s:      move_group  →  plans motions, sends to arm_controller

after 6s:      rviz2  →  visualizes + lets you send goals
"""

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction so we can resolve LaunchConfiguration values at runtime."""

    transport = LaunchConfiguration("transport").perform(context)   # "serial" or "wifi"
    esp_ip    = LaunchConfiguration("esp_ip").perform(context)
    esp_port  = LaunchConfiguration("esp_port").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)

    # ── Package paths ─────────────────────────────────────────────────────────
    moveit_pkg = get_package_share_directory("arm_moveit_conf_pkg")
    arm_pkg    = get_package_share_directory("robotic_arm_control")

    # ── Patch URDF transport params at launch time ────────────────────────────
    # We read the installed URDF, replace the four param values, and pass the
    # patched string directly to robot_state_publisher and ros2_control_node.
    # This avoids touching the source file and works whether you colcon-installed
    # or used --symlink-install.
    urdf_path = os.path.join(arm_pkg, "urdf", "URDF.urdf")
    with open(urdf_path, "r") as f:
        urdf_str = f.read()

    def set_param(xml: str, name: str, value: str) -> str:
        """Replace <param name="NAME">anything</param> with the given value."""
        return re.sub(
            rf'(<param name="{name}">)[^<]*(</param>)',
            rf'\g<1>{value}\g<2>',
            xml,
        )

    urdf_str = set_param(urdf_str, "transport",   transport)
    urdf_str = set_param(urdf_str, "serial_port", serial_port)
    urdf_str = set_param(urdf_str, "esp_ip",      esp_ip)
    urdf_str = set_param(urdf_str, "esp_port",    esp_port)

    robot_description = {"robot_description": urdf_str}

    # ── MoveIt config (kinematics, srdf, controllers) ─────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("URDF", package_name="arm_moveit_conf_pkg")
        .to_moveit_configs()
    )
    # Override robot_description with our patched URDF
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["robot_description"] = urdf_str

    ros2_controllers_yaml = os.path.join(moveit_pkg, "config", "ros2_controllers.yaml")
    rviz_config           = os.path.join(moveit_pkg, "config", "moveit.rviz")

    # ── 1. robot_state_publisher ──────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── 2. ros2_control_node ──────────────────────────────────────────────────
    # Loads ArmHardwareInterface → opens serial port or TCP socket.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
        ],
        output="screen",
    )

    # ── 3. joint_state_broadcaster ────────────────────────────────────────────
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    delayed_jsb = TimerAction(period=3.0, actions=[jsb_spawner])

    # ── 4. arm_controller ─────────────────────────────────────────────────────
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # ── 5. move_group ─────────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    delayed_move_group = TimerAction(period=5.0, actions=[move_group_node])

    # ── 6. RViz ───────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config_dict],
    )

    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    return [
        rsp_node,
        ros2_control_node,
        delayed_jsb,
        delay_arm_controller,
        delayed_move_group,
        delayed_rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            "transport",
            default_value="serial",
            description="Communication transport: 'serial' (USB) or 'wifi' (TCP)",
            choices=["serial", "wifi"],
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port device (used when transport=serial)",
        ),
        DeclareLaunchArgument(
            "esp_ip",
            default_value="192.168.1.105",
            description="ESP IP address (used when transport=wifi)",
        ),
        DeclareLaunchArgument(
            "esp_port",
            default_value="8888",
            description="ESP TCP port (used when transport=wifi)",
        ),
        # ── Everything else resolved at runtime ───────────────────────────────
        OpaqueFunction(function=launch_setup),
    ])
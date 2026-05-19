#!/usr/bin/env python3
"""
real_arm.launch.py
==================
Launches the arm with the real ArmHardwareInterface (ESP8266 over /dev/ttyUSB0).

Starts in order:
  1. robot_state_publisher       — publishes TF from URDF
  2. ros2_control_node           — loads ArmHardwareInterface, opens serial port
  3. joint_state_broadcaster     — after 3 s, reads STATE from ESP → /joint_states
  4. arm_controller              — after joint_state_broadcaster is active
  5. move_group                  — after 5 s, MoveIt planning
  6. rviz2                       — after 6 s, with MoveIt plugin

Place this file in:  arm_moveit_conf_pkg/launch/real_arm.launch.py

Before launching:
  - Make sure URDF has ArmSystem (ArmHardwareInterface) active, NOT GazeboSimSystem
  - ESP8266 must be flashed and connected on /dev/ttyUSB0
  - Build: colcon build --symlink-install
  - Launch: ros2 launch arm_moveit_conf_pkg real_arm.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────────────
    moveit_pkg = get_package_share_directory("arm_moveit_conf_pkg")
    arm_pkg    = get_package_share_directory("robotic_arm_control")

    # ── Load URDF and config files ────────────────────────────────────────────
    # MoveItConfigsBuilder reads the URDF from arm_moveit_conf_pkg.
    # It does NOT override the hardware plugin — whatever is in the URDF is used.
    # Make sure ArmSystem (ArmHardwareInterface) is the active <ros2_control> block.
    moveit_config = (
        MoveItConfigsBuilder("URDF", package_name="arm_moveit_conf_pkg")
        .to_moveit_configs()
    )

    ros2_controllers_yaml = os.path.join(moveit_pkg, "config", "ros2_controllers.yaml")
    rviz_config           = os.path.join(moveit_pkg, "config", "moveit.rviz")

    # ── 1. robot_state_publisher ─────────────────────────────────────────────
    # Publishes TF from URDF. Reads joint angles from /joint_states.
    # No use_sim_time here — this is real hardware on wall clock.
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── 2. ros2_control_node ─────────────────────────────────────────────────
    # The controller manager. Loads ArmHardwareInterface → opens /dev/ttyUSB0.
    # If this fails, check: ls -la /dev/ttyUSB0  and  sudo chmod 666 /dev/ttyUSB0
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # gives it the URDF with plugin info
            ros2_controllers_yaml,            # controller types and joint config
        ],
        output="screen",
    )

    # ── 3. joint_state_broadcaster ───────────────────────────────────────────
    # Reads position_state_[] from hardware interface → publishes /joint_states.
    # Delayed 3 s to give ros2_control_node time to start and open the serial port.
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

    # ── 4. arm_controller ────────────────────────────────────────────────────
    # JointTrajectoryController. Spawned only after joint_state_broadcaster
    # successfully activates — same pattern as gazebo.launch.py.
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

    # ── 5. move_group ────────────────────────────────────────────────────────
    # MoveIt planning server. Connects to arm_controller via FollowJointTrajectory.
    # Delayed 5 s to give controllers time to become active first.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    delayed_move_group = TimerAction(period=5.0, actions=[move_group_node])

    # ── 6. RViz with MoveIt plugin ───────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict()],
    )

    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    # ── Launch description ───────────────────────────────────────────────────
    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        delayed_jsb,            # waits 3 s then spawns joint_state_broadcaster
        delay_arm_controller,   # spawns arm_controller after jsb exits cleanly
        delayed_move_group,     # waits 5 s then starts move_group
        delayed_rviz,           # waits 6 s then starts rviz
    ])
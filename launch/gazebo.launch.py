#!/usr/bin/env python3
"""
Launch file to simulate the arm in Gazebo Harmonic
Substitutes controller config path in URDF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directory
    pkg_name = 'robotic_arm_control'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'URDF.urdf')
    controller_config = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')
    
    # Read URDF and inject the controller config path into the gz_ros2_control plugin
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Inject <parameters> into the gz_ros2_control gazebo plugin block
    robot_desc = robot_desc.replace(
        '<controller_manager_name>controller_manager</controller_manager_name>\n    </plugin>',
        f'<controller_manager_name>controller_manager</controller_manager_name>\n      <parameters>{controller_config}</parameters>\n    </plugin>'
    )
    
    # Set Gazebo resource path to find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )
    
    # Set Gazebo plugin path
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )
    
    # Start Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    
    # Bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm_robot',
            # '-z', '0.5'
        ],
        output='screen'
    )
    
    # Delay spawn by 5 seconds to let Gazebo and controller manager fully start
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    
    # Joint State Broadcaster - spawned by gz_ros2_control plugin
    # We just wait for it to be available
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    
    # Delay controllers after spawn with longer delay
    delay_joint_state_broadcaster = TimerAction(
        period=8.0,  # Wait 8 seconds after launch
        actions=[joint_state_broadcaster_spawner]
    )
    
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        gazebo,
        robot_state_publisher,
        bridge_clock,
        delayed_spawn,
        delay_joint_state_broadcaster,
        delay_arm_controller
    ])  
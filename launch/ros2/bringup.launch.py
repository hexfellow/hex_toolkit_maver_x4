#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import xacro
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # arg
    visual_flag = DeclareLaunchArgument(
        'visual_flag',
        default_value='true',
    )
    sim_time_flag = DeclareLaunchArgument(
        'sim_time_flag',
        default_value='false',
    )
    hardware_flag = DeclareLaunchArgument(
        'hardware_flag',
        default_value='true',
    )
    
    # visual
    urdf_file_path = FindPackageShare('hex_toolkit_maver_x4').find(
            'hex_toolkit_maver_x4') + '/urdf/maver_x4_ros2.xacro'
    rviz_file_path = FindPackageShare('hex_toolkit_maver_x4').find(
        'hex_toolkit_maver_x4') + '/config/ros2/display.rviz'
    
    visual_group = GroupAction(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('sim_time_flag'),
                    'robot_description':
                    xacro.process_file(urdf_file_path).toxml(),
                }],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                parameters=[{
                    'use_sim_time':
                    LaunchConfiguration('sim_time_flag'),
                }],
                arguments=['-d', rviz_file_path],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('visual_flag')),
    )

    # sim
    odom_sim_param = FindPackageShare('hex_toolkit_maver_x4').find(
        'hex_toolkit_maver_x4') + '/config/ros2/odom_sim.yaml'
    sim_group = GroupAction(
        [
            Node(
                package='hex_toolkit_general_chasssis',
                executable='odom_sim',
                name='odom_sim',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('sim_time_flag'),
                    },
                    odom_sim_param,
                ],
                remappings=[
                    # subscribe
                    ('/cmd_vel', '/cmd_vel'),
                    # publish
                    ('/odom', '/odom'),
                ],
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('hardware_flag')),
    )
    
    # real
    bringup_param = FindPackageShare('hex_toolkit_maver_x4').find(
        'hex_toolkit_maver_x4') + '/config/ros2/bringup.yaml'
    real_group = GroupAction(
        [
            Node(
                package='robot_hardware_interface',
                executable='chassis_trans',
                name='hex_chassis',
                output="screen",
                emulate_tty=True,
                parameters=[bringup_param],
                remappings=[
                    # publish
                    ('/motor_states', '/motor_states'),
                    ('/real_vel', '/real_vel'),
                    # subscribe
                    ('/joint_ctrl', '/joint_ctrl'),
                    ('/cmd_vel', '/cmd_vel'),
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('hardware_flag')),
    )

    return LaunchDescription([
        # arg
        visual_flag,
        sim_time_flag,
        hardware_flag,
        # visual
        visual_group,
        # sim
        sim_group,
        # real
        real_group,
    ])
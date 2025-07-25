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


def generate_launch_description():
    # arg
    visual_flag = DeclareLaunchArgument(
        'visual_flag',
        default_value='true',
    )
    
    sim_flag = DeclareLaunchArgument(
        'sim_flag',
        default_value='false',
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
                    'use_sim_time': LaunchConfiguration('sim_flag'),
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
                    'use_sim_time': LaunchConfiguration('sim_flag'),
                }],
                arguments=['-d', rviz_file_path],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('visual_flag')),
    )

    bringup_sim = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('hex_toolkit_maver_x4'),
                'launch/ros2',
                'bringup_sim.launch.py'
            ]),
            condition=IfCondition(LaunchConfiguration('sim_flag'))
        )
    
    bringup_real = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('hex_toolkit_maver_x4'),
                'launch/ros2',
                'bringup_real.launch.py'
            ]),
            condition=UnlessCondition(LaunchConfiguration('sim_flag'))
        )

    return LaunchDescription([
        # arg
        visual_flag,
        sim_flag,
        # visual
        visual_group,
        # sim
        bringup_sim,
        # real
        bringup_real,
    ])
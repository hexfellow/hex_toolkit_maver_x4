#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 
        'launch', 
        'gazebo.launch.py'
    ])
    
    gazebo_param = PathJoinSubstitution([
        FindPackageShare('hex_toolkit_maver_x4'),
        'config/ros2',
        'gazebo.yaml'
    ])
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_path]),
            launch_arguments={
                'params_file': gazebo_param,
            }.items(),
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'maver_x4',
                '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'
            ],
        ),
    ])
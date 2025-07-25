#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_yaml = FindPackageShare('hex_toolkit_maver_x4').find(
        'hex_toolkit_maver_x4') + '/config/ros2/bringup.yaml'
    
    return LaunchDescription([
        Node(
            package='robot_hardware_interface',
            executable='chassis_trans',
            name='hex_chassis',
            output="screen",
            emulate_tty=True,
            parameters=[bringup_yaml],
            remappings=[
                # publish
                ('/motor_states', '/motor_states'),
                ('/real_vel', '/real_vel'),
                # subscribe
                ('/joint_ctrl', '/joint_ctrl'),
                ('/cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
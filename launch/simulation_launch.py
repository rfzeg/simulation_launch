#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Gazebo launch
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo world: configure as needed
    pkg_gazebo_world = get_package_share_directory('simulation_launch')
    world_relative_path = 'worlds'
    world_filename = 'empty.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_gazebo_world, world_relative_path, world_filename), ''],
            description='SDF world file'),

        gazebo,

        Node(package='simulation_launch', executable='spawn_entity_client.py',
             output='screen')
    ])

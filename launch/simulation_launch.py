#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Robot model: configure as needed
    robot_model_file = 'rrbot.xacro'
    robot_model_relative_path = 'urdf/'
    robot_model_package = 'rrbot_description'

    xacro_file = os.path.join(get_package_share_directory(
        robot_model_package), robot_model_relative_path, robot_model_file)
    assert os.path.exists(
        xacro_file), "The file "+str(robot_model_file)+" doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix(robot_model_package)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

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
             arguments=[robot_desc], output='screen'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}],
            output="screen"),
    ])

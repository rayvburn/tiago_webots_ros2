#!/usr/bin/env python

"""Launch Webots and the controller."""

import os

import launch
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

package_name = 'tiago_webots_ros2_navigation'

def generate_launch_description():
    package_dir = get_package_share_directory(package_name)

    world_file = LaunchConfiguration(
        'world_file',
        default='intralogistics.wbt'
    )

    # Webots with TIAGo Iron driver
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_driver'), 'launch', 'tiago_webots.launch.py')
        ), launch_arguments={
            'world_file': world_file
        }.items()
    )

    # Map Server Node
    map_server_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_navigation'), 'launch', 'map_server.launch.py')
        ),
        launch_arguments={}.items()
    )

    # Amcl Node
    amcl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_navigation'), 'launch', 'localization.launch.py')
        ),
        launch_arguments={}.items()
    )

    # Navigator Node
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_navigation'), 'launch', 'navigation.launch.py')
        ),
        launch_arguments={}.items()
    )

    # Rviz node
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_navigation'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('world_file', default_value=world_file),
        webots,
        map_server_node,
        amcl_node,
        navigation_node,
        rviz_node
    ])
#!/usr/bin/env python

"""Launch Webots and the controller."""

import os
import launch
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

package_name = 'tiago_webots_ros2_driver'

def generate_launch_description():
    world_dir = LaunchConfiguration(
        'world',
        default=os.path.join(
            get_package_share_directory('tiago_webots_ros2_driver'),
            'worlds',
            'intralogistics.wbt'
        )
    )

    # Webots with TIAGo Iron driver
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'webots_ros2_core'),
                'launch',
                'robot_launch.py'
            )
        ),
        launch_arguments=[
            ('package', package_name),
            ('executable', 'tiago_driver'),
            ('world', world_dir),
            ('output', 'screen')
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world',
            default_value=world_dir,
            description='directory of the world file (including file extension)'
        ),
        webots
    ])
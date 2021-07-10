#!/usr/bin/env python

"""Launch Webots and the controller."""

import os
import launch
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

package_name = 'tiago_webots_ros2_driver'

def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='intralogistics.wbt')

    # Webots with TIAGo Iron driver
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', package_name),
            ('executable', 'tiago_driver'),
            ('world', PathJoinSubstitution([get_package_share_directory(package_name), 'worlds', world_file])),
            ('output', 'screen')
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=world_file,
            description='name of the world file (including file extension)'
        ),
        webots
    ])
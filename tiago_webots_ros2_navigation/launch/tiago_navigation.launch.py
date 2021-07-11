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

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='false'
    )

    map_dir = LaunchConfiguration(
        'map_file',
        default=os.path.join(
            get_package_share_directory('tiago_webots_ros2_driver'),
            'resource',
            'map',
            'intralogistics.yaml'
        )
    )

    # param_dir = LaunchConfiguration(
    #     'params_file',
    #     default=os.path.join(
    #         get_package_share_directory(package_name),
    #         'config',
    #         'params.yaml')
    # )
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            'burger.yaml')
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('tiago_webots_ros2_navigation'),
    #     'rviz',
    #     'tiago_navigation.rviz'
    # )
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )


    # Webots with TIAGo Iron driver
    # webots = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('tiago_webots_ros2_driver'), 'launch', 'tiago_webots.launch.py')
    #     ), launch_arguments={
    #         'world_file': world_file
    #     }.items()
    # )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('world_file', default_value=world_file),
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        launch.actions.DeclareLaunchArgument('map_file', default_value=map_dir),

        # webots,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ])
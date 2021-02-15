#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controller."""

import os

import launch
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

package_name = 'tiago_webots_ros2'

def generate_launch_description():
    package_dir = get_package_share_directory(package_name)

    # Webots
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_differential_drive_node',
            'world': os.path.join(package_dir, 'worlds', 'intralogistics_2.wbt'),
            'node_parameters': os.path.join(package_dir, 'config', 'tiago.yaml'),
            'output': 'screen'
        }.items()
    )

    # TiagoRobot node
    tiago_robot_node = Node(
        package='tiago_webots_ros2',
        executable='robot_task_node',
        output='screen'
    )

    # Map Server Node
    map_file = os.path.join(package_dir, 'resources', 'map', 'intralogistics.yaml')
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Map Server node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_server_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name='map_server'),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN
                )),
                LogInfo(msg="[LifecycleLaunch] Map Server node is exiting.")
            ]
        )
    )

    # Rviz node
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default=True)
    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'odometry.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        webots,
        tiago_robot_node,
        map_server_node,
        activate_event,
        configure_event,
    #    shutdown_event,
        rviz
    ])
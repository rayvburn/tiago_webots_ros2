import os
import launch
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

package_name='tiago_webots_ros2_mapping'

def generate_launch_description():
    world_file = LaunchConfiguration(
        'world_file',
        default='intralogistics.wbt'
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true'
    )

    cartographer_config_dir = LaunchConfiguration(
        'package_cartographer_cfg',
        default=os.path.join(get_package_share_directory(package_name), 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='tiago_lds_2d.lua'
    )

    resolution = LaunchConfiguration(
        'resolution',
        default='0.05'
    )
    publish_period_sec = LaunchConfiguration(
        'publish_period_sec',
        default='1.0'
    )

    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=os.path.join(
            get_package_share_directory(package_name),
            'rviz',
            'tiago_cartographer.rviz'
        )
    )

    # Webots with TIAGo Iron driver
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_driver'), 'launch', 'tiago_webots.launch.py')
        ), launch_arguments={
            'world_file': world_file
        }.items()
    )

    # Cartographer
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tiago_webots_ros2_mapping'), 'launch', 'cartographer.launch.py')
        ), launch_arguments={
            'use_sim_time': use_sim_time,
            'cartographer_config_dir': cartographer_config_dir,
            'configuration_basename': configuration_basename,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec,
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # Rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=world_file,
            description='name of the world file (including file extension)'
        ),
        launch.actions.DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'
        ),
        launch.actions.DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'
        ),
        launch.actions.DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Rviz2 configuration file'
        ),
        webots,
        cartographer,
        rviz2
    ])
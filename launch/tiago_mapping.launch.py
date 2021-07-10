import os
import launch
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

package_name = 'tiago_webots_ros2'

def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='intralogistics.wbt')

    # Webots with TIAGo Iron driver
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'tiago_webots.launch.py')
        )
    )

    # Cartographer
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'cartographer.launch.py')
        )
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=world_file,
            description='name of the world file (including file extension)'
        ),
        webots,
        cartographer
    ])
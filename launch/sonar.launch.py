import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    sonar = LaunchConfiguration('sonar', default='true')
    device = LaunchConfiguration('device', default='')
    hold = os.path.join(get_package_share_directory('imagenex831l_ros2'),
        'cfg',
        'sonar.yaml'
        )
    config = LaunchConfiguration('config', default=hold)

    return LaunchDescription([
        Node(
            package='imagenex831l_ros2',
            executable='sonar_node.py',
            name='imagenex831l_ros2',
            parameters = [config, device],
            condition=LaunchConfigurationEquals('sonar', 'true')
        )
    ])

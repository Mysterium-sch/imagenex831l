import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('imagenex831l'),
        'cfg',
        'sonar.yaml'
        )

    return LaunchDescription([
        Node(
            package='imagenex831l',
            executable='sonar_node.py',
            name='imagenex831l',
            parameters = [config]
        )
    ])

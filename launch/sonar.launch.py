import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imagenex831l',
            executable='sonar_node.py',
            name='sonar',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=30
        )
    ])

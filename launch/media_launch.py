from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hsrb_display',
            executable='media_display',
            name='media_display',
            output='screen'
        )
    ])

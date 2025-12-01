from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Pfad zum Package ermitteln
    pkg_dir = get_package_share_directory('hsrb-display')

    # Pfad zum Python-Skript
    script_path = os.path.join(pkg_dir, 'hsrb-display', 'startup', 'start_subscriber.py')

    return LaunchDescription([
        Node(
            package='hsrb-display',
            executable='start_subscriber',
            output='screen'
        )
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fk_bringup'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(package='fk_auto', executable='auto_runner', name='auto_runner',
             parameters=[params], output='screen'),
    ])

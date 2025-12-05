"""Launch file for dobot action servers."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dobot_actions',
            executable='move_joints_server.py',
            name='move_joints_server',
            output='screen',
        ),
    ])

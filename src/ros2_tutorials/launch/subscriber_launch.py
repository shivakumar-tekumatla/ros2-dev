#!/usr/bin/env python3
"""Launch file for subscriber node only"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with subscriber node only"""
    
    return LaunchDescription([
        Node(
            package='ros2_tutorials',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen',
        ),
    ])

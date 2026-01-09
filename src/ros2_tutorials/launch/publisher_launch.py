#!/usr/bin/env python3
"""Launch file for publisher node only"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with publisher node only"""
    
    return LaunchDescription([
        Node(
            package='ros2_tutorials',
            executable='publisher_node',
            name='publisher_node',
            output='screen',
        ),
    ])

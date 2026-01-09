#!/usr/bin/env python3
"""Launch file for ros2_tutorials package"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with publisher and subscriber nodes"""
    
    return LaunchDescription([
        Node(
            package='ros2_tutorials',
            executable='publisher_node',
            name='publisher_node',
            output='screen',
        ),
        Node(
            package='ros2_tutorials',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen',
        ),
    ])

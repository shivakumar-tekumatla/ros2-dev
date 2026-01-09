#!/usr/bin/env python3
"""Simple ROS 2 Publisher Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """A simple publisher node that publishes messages every second"""

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Callback for the timer"""
        self.counter += 1
        msg = String()
        msg.data = f'Hello from ROS 2! Message #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    publisher_node = PublisherNode()
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Simple ROS 2 Subscriber Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """A simple subscriber node that listens to messages"""

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """Callback for incoming messages"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    subscriber_node = SubscriberNode()
    
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

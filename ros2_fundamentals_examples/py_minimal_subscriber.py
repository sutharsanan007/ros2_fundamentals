#! /usr/bin/env python3

"""
Description:
    This ROS 2 node subscribes to a topic and logs received "Hello World!" messages.
    
-------------------------------------------------------------
Publishing Topics:
    None
-------------------------------------------------------------
Subscription Topics:
    The channel containing the "Hello World!" messages.
    /py_example_topic (std_msgs/msg/String)
-------------------------------------------------------------
Author: Pins
Date: Jan 16, 2026
"""

import rclpy # ROS 2 Python client library
from rclpy.node import Node # Import the node class, used for creating ROS 2 nodes
from std_msgs.msg import String # Import the String message type from std_msgs package for receiving text messages

class MinimalPySubscriber(Node):
    """
    A minimal ROS 2 subscriber node that listens for "Hello World!" messages.
    """

    def __init__(self):
        """Create a custom ROS 2 node for subscribing to messages."""
        # Initialize the node with a name
        super().__init__('minimal_py_subscriber')
        
        # Create a subscriber on the topic with a queue size of 10 messages
        self.subscription = self.create_subscription(
            String,
            '/py_example_topic',
            self.listener_callback,
            10)
        
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        """Callback function executed when a message is received"""
        
        # Log the received message to the console
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPySubscriber node
    minimal_py_subscriber = MinimalPySubscriber()

    # Spin the node to keep it active and listening for messages
    rclpy.spin(minimal_py_subscriber)
    
    # Clean up and shut down the node when done
    minimal_py_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
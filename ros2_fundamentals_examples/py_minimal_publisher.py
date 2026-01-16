#! /usr/bin/env python3

"""
Description: This ROS 2 node periodically publishes "Hello World!" messages to a topic.

-------------------------------------------------------------
Publishing Topics:
    The channel containing the "Hello World!" messages.
    /py_example_topic (std_msgs/msg/String)
    
Subscription Topics:
    None
-------------------------------------------------------------
Author: Pins
Date: Jan 12, 2026
"""

import rclpy # ROS 2 Python client library
from rclpy.node import Node # Import the node class, used for creating ROS 2 nodes
from std_msgs.msg import String # Import the String message type from std_msgs package for publishing text messages

class MinimalPyPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes "Hello World!" messages.
    """

    def __init__(self):
        """Create a custom ROS 2 node for publishing messages."""
        # Initialize the node with a name
        super().__init__('minimal_py_publisher')
        
        # Create a publisher on the topic with a queue size of 10 messages
        self.publisher_1 = self.create_publisher(String, '/py_example_topic', 10)
        
        # Create a timer with a period of 0.5 seconds to trigger publishing a message
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize a counter for the messages
        self.i = 0
    
    def timer_callback(self):
        """Callback function executed periodically by the timer"""
        
        # Create a new String message object
        msg = String()
        
        # Set the data field of the message to "Hello World!" with a counter
        msg.data = f'Hello World! {self.i}'
        self.i += 1
        
        # Publish the message to the topic
        self.publisher_1.publish(msg)
        
        # Log the message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MinimalPyPublisher node
    minimal_py_publisher = MinimalPyPublisher()
    
    # Keep the node running and processing callbacks
    rclpy.spin(minimal_py_publisher)
    
    # Clean up and shut down the node when done
    minimal_py_publisher.destroy_node()
    rclpy.shutdown()

# This is required for Python scripts to run the main function    
if __name__ == '__main__':
    main()
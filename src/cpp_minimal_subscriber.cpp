/**
 * @file cpp_minimal_subscriber.cpp
 * @brief A minimal ROS 2 subscriber node in C++ that subscribes to String messages.
 * @author Pins
 * @date 2026-01-16
 * @version 1.0
 * @copyright Copyright (c) 2026 Pins. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp" // Include the main ROS 2 C++ client library header
#include "std_msgs/msg/string.hpp" // Include the standard String message type

using namespace std::chrono_literals; // Handles time literals like 500ms

class MinimalCppSubscriber : public rclcpp::Node // Define a class inheriting from rclcpp::Node
{
public:
  MinimalCppSubscriber() : Node("minimal_cpp_subscriber") // Constructor initializes the node with the name "minimal_subscriber"
  {
    // Create a subscriber that subscribes to std_msgs::msg::String messages on the "/cpp_example_topic" topic with a queue size of 10
    subscriber_ = create_subscription<std_msgs::msg::String>(
        "/cpp_example_topic", 10, std::bind(&MinimalCppSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscriber node has been started.");
  }

  void topic_callback(const std_msgs::msg::String & msg) const // Callback function for handling received messages
  {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str()); // Log the received message
  }

private:
  // Member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // Subscriber member variable
};

int main(int argc, char * argv[]) // Main function
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 client library

  auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>(); // Create an instance of the subscriber node
  rclcpp::spin(minimal_cpp_subscriber_node); // Spin the node to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 client library
  return 0; // Return success
}
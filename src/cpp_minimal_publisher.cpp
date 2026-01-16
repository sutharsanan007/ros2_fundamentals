/**
 * @file cpp_minimal_publisher.cpp
 * @brief A minimal ROS 2 publisher node in C++ that publishes String messages.
 * @author Pins
 * @date 2026-01-16
 * @version 1.0
 * @copyright Copyright (c) 2026 Pins. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp" // Include the main ROS 2 C++ client library header
#include "std_msgs/msg/string.hpp" // Include the standard String message type

using namespace std::chrono_literals; // Handles time literals like 500ms

class MinimalCppPublisher : public rclcpp::Node // Define a class inheriting from rclcpp::Node
{
public:
  MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0) // Constructor initializes the node with the name "minimal_publisher"
  {
    // Create a publisher that publishes std_msgs::msg::String messages on the "/cpp_example_topic" topic with a queue size of 10
    publisher_ = create_publisher<std_msgs::msg::String>(
        "/cpp_example_topic", 10);

    // Create a timer that calls the timer_callback function every 500 milliseconds
    timer_ = create_wall_timer(
      500ms, std::bind(&MinimalCppPublisher::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Publisher node has been started.");
  }

  void timer_callback() // Function called every time the timer triggers
  {
    auto message = std_msgs::msg::String(); // Create a new String message
    message.data = "Hello, world! " + std::to_string(count_++); // Set the message data with an incrementing count

    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str()); // Log the message being published

    publisher_->publish(message); // Publish the message
  }

private:
  // Member variables
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher member variable
  rclcpp::TimerBase::SharedPtr timer_; // Timer member variable
  size_t count_; // Counter for the number of messages published
};

int main(int argc, char * argv[]) // Main function
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 client library

  auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>(); // Create an instance of the publisher node
  rclcpp::spin(minimal_cpp_publisher_node); // Spin the node to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 client library
  return 0; // Return success
}
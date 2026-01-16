#!/bin/bash

# This script launches both the minimal publisher and subscriber nodes with cleanup handling.
cleanup() {
    echo "Restarting ROS 2 daemon to cleanup before shutting down..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0 # Kills all processes launched by this script
    exit
}

# Set trap to catch SIGINT (Ctrl+C) and call cleanup function
trap 'cleanup' SIGINT

# Launch the minimal publisher in the background (using &) and subscriber node in the foreground
echo "Launching minimal publisher and subscriber nodes..."
ros2 run ros2_fundamentals_examples py_minimal_publisher.py &

sleep 2 # Give some time for the publisher to start

ros2 run ros2_fundamentals_examples py_minimal_subscriber.py
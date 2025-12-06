#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/wsluser/ros2_ws/install/setup.bash

# Start Node
echo "Starting Node..."
ros2 run ros2_bridge human_bridge_node &
NODE_PID=$!
sleep 3

# Start Sender
echo "Starting Sender..."
# Adjust path to where src is mounted/located
python3 src/ros2_bridge/test_udp_sender.py &
SENDER_PID=$!
sleep 3

# Echo Topic
echo "Echoing /joint_states..."
ros2 topic echo /joint_states --once

# Cleanup
echo "Cleaning up..."
kill $SENDER_PID
kill $NODE_PID
wait $NODE_PID 2>/dev/null

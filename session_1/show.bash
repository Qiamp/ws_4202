#!/bin/bash
cd ~/ws_4202
. session_1/install/setup.bash

echo "========== 1. Starting Simulator =========="
ros2 run turtlesim turtlesim_node & sleep 6;

echo "========== 2. Starting turtle circle controller =========="
ros2 run turtle_control turtle_control_node & sleep 5;

echo "========== 2. Starting turtle pose show =========="
ros2 run turtle_control turtle_pose_show_node & sleep 5;

wait;
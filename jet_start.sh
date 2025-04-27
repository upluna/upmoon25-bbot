#!/bin/bash
echo Building ROS2 Package...
colcon build --packages-select bbot
source install/local_setup.bash

echo Launching Nodes...
ros2 launch bbot jet_launch.py

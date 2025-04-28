#!/bin/bash
echo Building ROS2 Package...
sudo colcon build --packages-select bbot
source install/setup.bash

echo Launching Nodes...
ros2 launch bbot jet_launch.py

#!/bin/bash

colcon build --packages-select bbot
source install/local_setup.bash
ros2 launch bbot jet_launch.py

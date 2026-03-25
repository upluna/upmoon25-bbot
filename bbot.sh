#!/bin/bash

colcon build --packages-select bbot
source install/local_setup.bash
ros2 run bbot bbot_driver_kb

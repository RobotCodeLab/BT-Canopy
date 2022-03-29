#!/bin/bash

# This script should be run from the directory where turtlebot package is located on the host computer. Not the dev-tools directory.

source install/setup.bash

export TURTLEBOT3_MODEL=burger

gnome-terminal --tab -- ./build/groot/Groot

sleep 2

gnome-terminal --tab -- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

sleep 10

gnome-terminal --tab -- ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

sleep 5

gnome-terminal --tab -- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

sleep 5

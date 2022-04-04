#!/bin/bash

IMAGE_NAME="btcanvas"
CONTAINER_NAME="btcanvas-container" # must be lowercase

ROS_VERSION="galactic"

SOURCE_ROS="source /opt/ros/$ROS_VERSION/setup.bash"
SOURCE_TURTLEBOT="source /root/turtlebot3_ws/install/setup.bash"
SOURCE_GAZEBO="source /usr/share/gazebo/setup.sh"

NETWORK_OPTIONS=""
# if '-p' or '--port_forward' is passsed, then set NETWORK_OPTIONS='--network="host"'
if [ "$1" == "-p" ] || [ "$1" == "--port_forward" ]; then
    NETWORK_OPTIONS="--network=host"
fi

function exec_command_in_new_tab() {

    echo "Executing command: $1"

    gnome-terminal --tab -- bash -c "docker exec $CONTAINER_NAME bash -c '$1'; bash"
}

# check if docker container is running using docker ps
if [ $(docker ps | grep $CONTAINER_NAME | wc -l) -eq 0 ]; then
    echo "Container $CONTAINER_NAME is not yet running"
    
    # check if image btwatcher exists
    if [ $(docker images | awk '{ print $1}' | grep $IMAGE_NAME | wc -l) -eq 0 ]; then
        echo "Image $IMAGE_NAME is not yet built"
        kill -INT $$
    fi

    # check if rocker installed
    if [ $(which rocker) -eq 0]; then

        echo "Rocker not installed- Install rocker @ https://github.com/osrf/rocker"
        kill -INT $$
    fi

    # check if nvidia-smi is installed
    if [ $(which nvidia-smi | wc -l) -eq 0 ]; then
        echo "nvidia card not available or nvidia-smi is not installed"
        echo "Starting rocker container using integrated graphics"
        gnome-terminal --tab -- rocker --x11 --devices /dev/dri/card0 $NETWORK_OPTIONS --name $CONTAINER_NAME  $IMAGE_NAME

    else
        echo "nvidia card available"
        echo "Starting rocker container with nvidia card"
        echo "rocker --x11 --nvidia $NETWORK_OPTIONS --name $CONTAINER_NAME  $IMAGE_NAME "
        gnome-terminal --tab -- rocker --x11 --nvidia $NETWORK_OPTIONS --name $CONTAINER_NAME  $IMAGE_NAME

    fi

    # loop until container with name $CONTAINER_NAME is running
    while [ $(docker ps | grep $CONTAINER_NAME | wc -l) -eq 0 ]; do
        echo "Waiting for container $CONTAINER_NAME to start"
        sleep 1
    done

    sleep 1
fi

echo "Executing commands in container $CONTAINER_NAME"

exec_command_in_new_tab "$SOURCE_ROS && $SOURCE_TURTLEBOT && $SOURCE_GAZEBO && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

sleep 10

exec_command_in_new_tab "$SOURCE_ROS && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"

sleep 10

exec_command_in_new_tab "$SOURCE_ROS && $SOURCE_TURTLEBOT && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True"

sleep 2

exec_command_in_new_tab "/root/turtlebot3_ws/build/groot/Groot"

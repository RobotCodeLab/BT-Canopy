#!/bin/bash

IMAGE_NAME="btcanvas"
CONTAINER_NAME="btcanvas-container" # must be lowercase

ROS_VERSION="galactic"

# SOURCE_ROS="source /opt/ros/$ROS_VERSION/setup.bash"
# SOURCE_TURTLEBOT="source /root/turtlebot3_ws/install/setup.bash"
SOURCE_BASHRC=". /root/.bashrc"


function exec_command_in_new_tab() {

    echo "Executing command: $1"

    # gnome-terminal --tab -- bash -c "docker exec -it $CONTAINER_NAME $1; bash"
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

        gnome-terminal --tab -- rocker --x11 --devices /dev/dri/card0  --name $CONTAINER_NAME $IMAGE_NAME

    else
        echo "nvidia card available"
        echo "Starting rocker container with nvidia card"

        gnome-terminal --tab -- rocker --x11 --nvidia --name $CONTAINER_NAME $IMAGE_NAME
    fi
fi

echo "Executing commands in container $CONTAINER_NAME"

exec_command_in_new_tab "$SOURCE_BASHRC && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

sleep 10

exec_command_in_new_tab "$SOURCE_BASHRC && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"

sleep 10

exec_command_in_new_tab "$SOURCE_BASHRC && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True"

sleep 2

exec_command_in_new_tab "/root/turtlebot3_ws/build/groot/Groot"

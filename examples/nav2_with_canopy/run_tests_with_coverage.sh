#!/bin/bash

# Variables
IMAGE_NAME="$1"
CONTAINER_NAME="btcanvas-container" # must be lowercase
WS="nav2_ws"
ROS_VERSION="iron"

# Source commands.
SOURCE_ROS="source /opt/ros/$ROS_VERSION/setup.bash"
SOURCE_WS="source /root/$WS/install/setup.bash"
SOURCE_GAZEBO="source /usr/share/gazebo/setup.sh"

# If '-p' or '--port_forward' is passsed, then set NETWORK_OPTIONS='--network="host"'
NETWORK_OPTIONS=""
if [ "$1" = "-p" ] || [ "$1" = "--port_forward" ]; then
    NETWORK_OPTIONS="--network=host"
fi

function exec_docker() {
    echo "Executing command in Docker container: $1"
    tmux new-session -d "docker exec $CONTAINER_NAME bash -c '$1'; bash"
}

# Check if the Docker container is running using docker ps.
if [ $(docker ps | grep $CONTAINER_NAME | wc -l) -eq 0 ]; then
    echo "Container $CONTAINER_NAME is not yet running"
    
    # Check if the image (with image name) exists.
    if [ $(docker images | awk '{ print $1}' | grep $IMAGE_NAME | wc -l) -eq 0 ]; then
        echo "Image $IMAGE_NAME is not yet built"
        kill -INT $$
    fi

    # Check if rocker is installed.
    if [ $(which rocker) -eq 0 ]; then
        echo "Rocker not installed - Install rocker @ https://github.com/osrf/rocker"
        kill -INT $$
    fi

    # Check if nvidia-smi is installed.
    if [ $(which nvidia-smi | wc -l) -eq 0 ]; then
        echo "Nvidia card not available or nvidia-smi is not installed"
        echo "Starting rocker container using integrated graphics"
        echo "rocker --x11 --devices /dev/dri/card0 $NETWORK_OPTIONS --name $CONTAINER_NAME $IMAGE_NAME"
        tmux new-session -d "rocker --x11 --devices /dev/dri/card0 $NETWORK_OPTIONS --name $CONTAINER_NAME $IMAGE_NAME"
    else
        echo "Nvidia card available"
        echo "Starting rocker container with nvidia card"
        echo "rocker --x11 --nvidia $NETWORK_OPTIONS --name $CONTAINER_NAME $IMAGE_NAME"
        tmux new-session -d "rocker --x11 --nvidia $NETWORK_OPTIONS --name $CONTAINER_NAME $IMAGE_NAME"
    fi

    # Loop until container with name $CONTAINER_NAME is running.
    while [ $(docker ps | grep $CONTAINER_NAME | wc -l) -eq 0 ]; do
        echo "Waiting for container $CONTAINER_NAME to start"
        sleep 1
    done

    sleep 1
fi

echo "Executing Nav2 with Canopy logging in container $CONTAINER_NAME"
exec_docker "$SOURCE_ROS && $SOURCE_WS && $SOURCE_GAZEBO && ros2 launch bt_canopy_bringup monitor.launch.py"
sleep 2
exec_docker "$SOURCE_ROS && $SOURCE_WS && $SOURCE_GAZEBO && cd /root/nav2_ws/build/nav2_system_tests/ && ctest -V; lcov --capture --rc lcov_branch_coverage=1 --directory ../ --output-file lcov_report.info"


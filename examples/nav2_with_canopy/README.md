# Nav2 with Canopy Example

This Dockerfile starts with a base ROS2 Iron image and then clones and builds a [slightly modified version of Navigation2](https://github.com/RobotCodeLab/navigation2/tree/iron), [BT-Canopy](https://github.com/RobotCodeLab/BT-Canopy), and [Groot](https://github.com/BehaviorTree/Groot).

## Building the Docker Image

From inside the `nav2_with_canopy` folder, run `docker build . -t nav2_with_canopy`.

Note 1: This may take a while since we have to build Nav2 from source.

Note 2: If you wish to disable the Gazebo client (the GUI) for the test suite, uncomment line 16 in the Dockerfile before building.

## Launching the Docker Image

You'll probably want to be able to view graphical applications from inside your docker container. In fact, headless Gazebo might complain if it's run inside a docker without a display. BT-Canopy was developed using the convenient [rocker](https://github.com/osrf/rocker) tool which we recommend.

To run the container using integrated graphics:

```bash
rocker --x11 --devices /dev/dri/card0 --name btcanvas-container nav2_with_canopy`
```

To run the container using nvidia graphics:

```bash
rocker --x11 --nvidia --name btcanvas-container nav2_with_canopy
```

## Launching Nav2 with Canopy (From Inside the Container)

To launch Canopy logging:

```bash
ros2 launch bt_canopy_bringup monitor.launch.py
```

To launch a Nav2 test world (in another tab):

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

To launch Groot monitoring (in another tab):

```bash
groot
```

Note: Groot monitoring won't connect until after you set the initial pose estimate in Rviz.

## Running the Container *and* Nav2 Test Suite

For our Nav2 case study, we ran the Nav2 test suite 10 times with Canopy logging. To launch a container and automatically start the test suite with Canopy logging, you can use the included `start_container_and_run_tests.sh` script. This script utilizes rocker.

```bash
bash run_tests_with_coverage.sh nav2_with_canopy
```

Note: The entire test suite runs in approximately less than 30 minutes.

## Copying Output Files

For our case study, we manually saved Canopy's csv files to our host machine using `docker cp`.

```bash
docker cp btcanvas-container:/root/nav2_ws/canopy_navigate_through_poses_w_replanning_and_recovery.csv ./
docker cp btcanvas-container:/root/nav2_ws/canopy_navigate_to_poses_w_replanning_and_recovery.csv ./
docker cp btcanvas-container:/root/nav2_ws/canopy_navigate_to_pose_w_replanning_and_recovery.csv ./
```


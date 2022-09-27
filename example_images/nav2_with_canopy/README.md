This Dockerfile starts with a base ROS2 Galactic image and then clones and builds a [slightly modified version of navigation2](https://github.com/RobotCodeLab/navigation2/tree/galactic), [BT-Canopy](https://github.com/RobotCodeLab/BT-Canopy), and [Groot](https://github.com/BehaviorTree/Groot) 

# To build the docker image
(from inside the nav2_with_canopy folder) `docker build -t nav2_with_canopy .`

note1: This may take a while since we have to build nav2 from source

note2: if you wish to disable the gazebo client (the gui) for the test suite, uncomment line 15 in the dockerfile before building.

# To launch the docker image
You'll probably want to be able to view graphical applications from inside your docker container. In fact, headless gazebo might complain if it's run inside a docker without a display (needs verficiation). BT-Canopy was developed using the convenient [rocker](https://github.com/osrf/rocker) tool which we recommend. 

To run the container using integrated graphics:
`rocker --x11 --devices /dev/dri/card0 --name btcanvas-container nav2_with_canopy`

To run the container using nvidia graphics:
`rocker --x11 --nvidia --name btcanvas-container nav2_with_canopy`

# nav2 with canopy (from inside container)

To launch Canopy logging:

`ros2 launch bt_canopy_bringup monitor.launch.py`

To launch a nav2 test world (in another tab):

`export TURTLEBOT3_MODEL=waffle`
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models`

`ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`

To launch groot monitoring (in another tab):

`groot`

note: groot monitoring won't connect until after you set the initial pose estimate in Rviz

# start new container *and* run nav2 system tests with canopy monitoring

For our Nav2 case study, we ran the nav2 test suite 10 times with canopy logging. To launch a container and automatically start the test suite with canopy logging, you can use the included start_container_and_run_tests.sh script. This script utilizes rocker. 

`bash start_container_and_run_tests.sh nav2_with_canopy`

note: For our case study, we manually saved Canopy's csv files to our host machine using `docker cp`. The entire test suite runs in approximatly 20 minutes. 
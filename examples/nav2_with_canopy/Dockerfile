FROM osrf/ros:iron-desktop

ENV ROS_DISTRO=iron

# Install Groot dependencies + turtlebot3 for nav2 sandbox
RUN apt-get update &&\
    apt-get install -y qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \ 
    ros-$ROS_DISTRO-turtlebot3*

WORKDIR /root/nav2_ws/src
RUN git clone -b $ROS_DISTRO https://github.com/RobotCodeLab/navigation2.git
# Uncomment the line below to disable gazebo client (The graphical interface) for test suite
# RUN cd navigation2 && git reset --hard 4be5f56f705cb31cd7db207b251a7cc702465d4c
RUN git clone https://github.com/RobotCodeLab/BT-Canopy.git
RUN git clone https://github.com/BehaviorTree/Groot.git
WORKDIR /root/nav2_ws

RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    apt-get update &&\
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO &&\
    colcon build --symlink-install

# Dist-upgrade is recommended for ROS dockers. See https://github.com/ros2/rosidl/issues/680
RUN apt-get update && \
    apt-get dist-upgrade -y 

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Source Gazebo (shortens bringup time)
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'echo "ran source /usr/share/gazebo/setup.sh"' >> ~/.bashrc

# Source ROS2
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> ~/.bashrc
RUN echo 'echo "ran source /opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

# Source the packages we built with colcon
RUN echo "source /root/nav2_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo 'echo "ran source /root/nav2_ws/install/local_setup.bash"' >> ~/.bashrc

# Add symbolic link for running groot to path
RUN ln -s /root/nav2_ws/build/groot/Groot /usr/local/bin/groot

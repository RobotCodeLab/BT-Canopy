FROM osrf/ros:galactic-desktop

ENV ROS_DISTRO=galactic

# Install Groot dependencies
RUN apt-get update &&\
    apt-get install -y qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev

WORKDIR /root/nav2_ws/src
RUN git clone -b $ROS_DISTRO https://github.com/hodgespodge/navigation2.git
RUN git clone https://github.com/hodgespodge/BT-Canopy.git
RUN git clone https://github.com/BehaviorTree/Groot.git
WORKDIR /root/nav2_ws


RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    apt-get update &&\
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO &&\
    colcon build --symlink-install

# Dist-upgrade is recommended for ROS dockers. See https://github.com/ros2/rosidl/issues/680
RUN apt-get update &&\
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


FROM osrf/ros:foxy-desktop

ENV ROS_DISTRO=foxy

RUN apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-py-trees-ros*
    
RUN apt-get update &&\
    #apt-get dist-upgrade -y &&\
    apt-get install vim -y
    
WORKDIR /root/pytrees_ws/src
RUN git clone https://github.com/hodgespodge/BT-Canopy.git
WORKDIR /root/pytrees_ws

# easier to touch colcon ignore than --packages-select for all packages
RUN touch /root/pytrees_ws/src/BT-Canopy/src/behaviortree_cpp_v3_ros2_publisher/COLCON_IGNORE

RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    apt-get update &&\
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO &&\
    colcon build --symlink-install

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Add ros source command to bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'echo "ran source /opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

RUN echo "source /root/pytrees_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo 'echo "ran source /root/pytrees_ws/install/local_setup.bash"' >> ~/.bashrc

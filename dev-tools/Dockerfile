# From here: https://github.com/hodgespodge/ros-robot-docker-images/tree/main/turtlebot-galactic
FROM turtlebot-galactic 
WORKDIR /root

RUN git clone https://github.com/hodgespodge/BTWatcher.git
WORKDIR /root/BTWatcher

RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    rosdep install --from-paths src --ignore-src &&\
    colcon build --symlink-install

RUN apt-get update && apt-get install -y vim libzmq3-dev

RUN echo 'source ~/BTWatcher/install/setup.bash' >> ~/.bashrc
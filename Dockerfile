FROM turtlebot-galactic
WORKDIR /root

RUN git clone https://github.com/hodgespodge/BTWatcher.git
WORKDIR /root/BTWatcher

RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    rosdep install --from-paths src --ignore-src &&\
    colcon build --symlink-install

RUN echo 'source ~/BTWatcher/install/setup.bash' >> ~/.bashrc
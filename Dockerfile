FROM osrf/ros:kinetic-desktop

RUN apt-get update

RUN apt-get install -y sudo \
                       wget \
                       lsb-release \
                       mesa-utils

WORKDIR /root

RUN /bin/bash -c "mkdir -p catkin_ws/src"

RUN cd catkin_ws/src && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"

RUN cd catkin_ws && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

RUN cd catkin_ws/src && git clone https://github.com/amslabtech/CATS --depth=1

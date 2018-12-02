FROM osrf/ros:kinetic-desktop

RUN apt-get update

RUN apt-get install -y sudo \
                       wget \
                       lsb-release \
                       mesa-utils

WORKDIR /root

# ROS setting
RUN /bin/bash -c "mkdir -p catkin_ws/src"

RUN cd catkin_ws/src && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace"

RUN cd catkin_ws && /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

# PCL installation
RUN apt-get purge libpcl*

RUN apt-get install -y libflann-dev

RUN git clone -b pcl-1.8.0rc2 https://github.com/PointCloudLibrary/pcl --depth=1 pcl-trunk

WORKDIR pcl-trunk

RUN mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j4 && make install && \
    make clean

# clone repository
WORKDIR /root

RUN cd catkin_ws/src && git clone https://github.com/amslabtech/CATS --depth=1

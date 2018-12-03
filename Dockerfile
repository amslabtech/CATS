FROM kazukitakahashi/ros_kinetic_pcl1.8

# clone repository
WORKDIR /root

RUN cd catkin_ws/src && git clone https://github.com/amslabtech/CATS --depth=1


#!/bin/bash

distro=$ROS_DISTRO
ros_ws=$ROS_WORKSPACE

source /opt/ros/${distro}/setup.bash
source ${ros_ws}/devel/setup.bash

source ${HOME}/.bashrc

gnome-terminal -e "/opt/ros/${distro}/bin/roscore" --geometry=50x12+0+0 &
sleep 3s

## tfrog
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch cats cats_vehicle.launch" --geometry=50x12+0+250 &

## imu
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch xsens_driver xsens.launch" --geometry=50x12+0+500 &

## velodyne
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch velodyne_pointcloud 32e_points.launch" --geometry=50x12+0+750 &

## hokuyo
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch cats front_laser.launch" --geometry=50x12+500+0 &
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch cats rear_laser.launch" --geometry=50x12+500+250 &

## joy
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch cats cats_ps4.launch" --geometry=50x12+500+500 &

## realsense
gnome-terminal -e "/${HOME}/realsense_docker/run_with_nvidia_docker.sh" --geometry=50x12+500+750 &

## usb cam
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch usb_cam usb_cam-test.launch" --geometry=50x12+1000+0 &

## tf
gnome-terminal -e "/opt/ros/${distro}/bin/roslaunch cats robot_state.launch" --geometry=50x12+1000+0 &

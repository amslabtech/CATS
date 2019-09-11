#!/bin/bash


source /opt/ros/kinetic/setup.bash
source ${HOME}/ros_catkin_ws/devel/setup.bash

source ${HOME}/.bashrc

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 3s

## tfrog
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats cats_vehicle.launch" --geometry=50x12+0+250 &

## imu
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch xsens_driver xsens_driver.launch" --geometry=50x12+0+500 &

## velodyne
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch velodyne_pointcloud 32e_points.launch" --geometry=50x12+0+750 &

## hokuyo
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats front_laser.launch" --geometry=50x12+500+0 &
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats rear_laser.launch" --geometry=50x12+500+250 &

## joy
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun joy joy_node" --geometry=50x12+500+500 &
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch motion_decision motion_decision.launch --screen" --geometry=50x12+500+500 &

## realsense
gnome-terminal -e "/${HOME}/realsense_docker/run_with_nvidia_docker.sh" --geometry=50x12+500+750 &

## usb cam
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats front_cam.launch" --geometry=50x12+1000+0 &

## tf
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats robot_state.launch" --geometry=50x12+1000+0 &

#!/bin/sh

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=40x12+0+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats gazebo.launch" --geometry=40x12+450+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats teleop_joy.launch" --geometry=40x12+850+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats rviz.launch" --geometry=40x12+1250+0

sleep 1s

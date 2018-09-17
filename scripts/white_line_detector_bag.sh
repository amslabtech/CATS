#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=45x12+0+0 &
sleep 0.5s

#gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ../config/tsukuba_cats.rviz" --geometry=45x12+475+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun cats white_line_detector.py" --geometry=45x12+895+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw" --geometry=45x12+1315+0 &
sleep 0.5s

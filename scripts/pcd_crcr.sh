#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=45x12+0+0 &
sleep 1.0s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ../config/pcd_crcr.rviz" --geometry=45x12+475+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats pcd_crcr.launch" --geometry=45x12+895+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun pcl_ros pcd_to_pointcloud /home/amsl/map/kakunin_soukou_0.5-2.pcd _frame_id:=/map" --geometry=45x12+1315+0 &
sleep 0.5s

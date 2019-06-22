# CATS

[![Build Status](https://travis-ci.org/amslabtech/CATS.svg?branch=master)](https://travis-ci.org/amslabtech/CATS)

## Environments
- Ubuntu 16.04LTS & ROS Kinetic
- Ubuntu 18.04LTS & ROS Melodic

## Requirements
```
sudo apt-get install ros-<distro>-navigation
sudo apt-get install ros-<distro>-velodyne-description
sudo apt-get install ros-<distro>-gazebo-plugins
```

## Hardware
- Velodyne Lidar HDL-32e
- Intel RealSense Depth Camera D435
- Xsens
- T-frog
- Hokuyo UTM-30LX

## How to Use
launch hardware driver
```
roscd cats/scripts
./cats_hardware.sh
```

open gazebo simulator
```
roslaunch cats gazebo.launch
```

## navigation sample
First you need to make map by SLAM(gmapping)
```
roslaunch cats gmapping.launch
```

Second you can navigation to luanch below scripts
```
roslaunch cats test.launch
roslaunch cats move_base.lauch
rosrun cats sending_goal
```

#!/bin/bash

echo "=== pcd_crcr ==="

pcd="pcd"

while :
do
  read -e -p "Enter INPUT file name (/home/hoge/dir/xxxxx.pcd):" INPUT
  if [ ${INPUT##*.} = "pcd" ]; then
    INPUT="$(eval "echo $INPUT")"
    echo "Input file is "$INPUT
    break
  else
    echo -e "Error : It is not a .pcd file!\n"
  fi
done

while :
do
  read -e -p "Enter OUTPUT file name(/home/hoge/dir/xxxxx.pcd):" OUTPUT
  if [ ${OUTPUT##*.} = "pcd" ]; then
    OUTPUT="$(eval "echo $OUTPUT")"
    echo "Output file is "$OUTPUT
    break
  else
    echo -e "Error : It is not a .pcd file!\n"
  fi
done

OUTPUT="${OUTPUT%.*}"

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=45x12+0+0 &
sleep 1.0s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ../config/pcd_crcr.rviz" --geometry=45x12+475+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch cats pcd_crcr.launch output_file_name:=$OUTPUT" --geometry=45x12+895+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun pcl_ros pcd_to_pointcloud $INPUT _frame_id:=/map" --geometry=45x12+1315+0 &
sleep 0.5s

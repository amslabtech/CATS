#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <std_msgs/Empty.h>

int main (int argc, char** argv){
  ros::init(argc,argv,"keyboard_interface");
  ros::NodeHandle n;

  ros::Publisher restart_pub = n.advertise<std_msgs::Empty>("/pause/restart", 1);

  std::cout << "------keyboard interface---------" << std::endl;

  ros::Rate loop_rate(10);

  while(ros::ok()){
    std::string str;
    getline(std::cin, str);
    if(str == " "){
      std::cout << "send restart command" << std::endl;
      std_msgs::Empty msg;
      restart_pub.publish(msg);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

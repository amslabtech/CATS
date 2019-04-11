#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom;
bool odom_received = false;

void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::PoseWithCovarianceStamped amcl = *msg;
  odom.header = amcl.header;
  odom.child_frame_id = "laser";
  odom.pose.pose = amcl.pose.pose;
  odom_received = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl2pose");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/pose",100);
  ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose",100, amcl_callback);	
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
	if(odom_received){
	  odom_pub.publish(odom);
	}
	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::Joy joy_data;

const double MAX_VELOCITY = 2.0;
const double MAX_ANGULAR_VELOCITY = 4.0;

void joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
  joy_data = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh;

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/infant3/diff_drive_controller/cmd_vel", 100);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 100, joy_callback);

  geometry_msgs::Twist velocity;

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!joy_data.axes.empty()){
      velocity.linear.x = MAX_VELOCITY * joy_data.axes[1];
      velocity.angular.z = MAX_ANGULAR_VELOCITY * joy_data.axes[0];
    }
    velocity_pub.publish(velocity);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


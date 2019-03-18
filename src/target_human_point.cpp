/*探索対象の位置をpublish
 *
 * author : K.Takahashi
 *
 */
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

class TargetHumanPoint
{
public:
  TargetHumanPoint(ros::NodeHandle, ros::NodeHandle);

  void process(void);
  void angle_callback(const std_msgs::Float32ConstPtr&);
  void scan_callback(const sensor_msgs::LaserScanConstPtr&);

private:
  ros::NodeHandle nh;
  ros::NodeHandle local_nh;
  ros::Publisher pose_pub;
  ros::Subscriber angle_sub;
  ros::Subscriber scan_sub;
  sensor_msgs::LaserScan scan;
  bool scan_flag;
  float angle;
  bool angle_flag;
  geometry_msgs::PoseStamped pt;
  float D_CAMERA;// velodyne to camera offset
};

int main (int argc, char** argv){
  ros::init(argc,argv,"target_human_point");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  TargetHumanPoint target_human_point(nh, local_nh);
  target_human_point.process();

  return 0;
}

TargetHumanPoint::TargetHumanPoint(ros::NodeHandle _nh, ros::NodeHandle _local_nh)
{
  nh = _nh;
  local_nh = _local_nh;
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/waypoint/add/no_id", 10);
  angle_sub = nh.subscribe("/target/angle", 1, &TargetHumanPoint::angle_callback, this);
  scan_sub = nh.subscribe("/velodyne_scan", 1, &TargetHumanPoint::scan_callback, this);
  local_nh.param("D_CAMERA", D_CAMERA, {0.135});
  scan_flag = false;
  angle_flag = false;
  pt.pose.orientation.w = 1.0;
}

void TargetHumanPoint::process(void)
{
  ros::Rate loop_rate(10);

  std::cout << "=== target_human_point ===" << std::endl;

  while(ros::ok()){
    if(scan_flag && angle_flag){
      int scan_size = scan.ranges.size();
      float min_diff = 100.0;
      float _angle = 0;
      for(int i=scan_size * 0.25;i<scan_size * 0.75;i++){
        float scan_angle = scan.angle_min + i * scan.angle_increment;
        // velodyne座標の角度と距離からカメラ座標の角度を求める
        float dist = scan.ranges[i];
        float theta = atan((dist * sin(scan_angle)) / ((dist * cos(scan_angle) - D_CAMERA)));
        float diff = fabs(theta - angle);
        if(min_diff > diff){
          min_diff = diff;
          _angle = scan_angle;
        }
      }
      int index = (_angle - scan.angle_min) / scan.angle_increment;
      float range = scan.ranges[index];
      if(!std::isinf(range)){
        pt.header = scan.header;
        pt.pose.position.x = range * cos(_angle);
        pt.pose.position.y = range * sin(_angle);
        //std::cout << pt.pose.position.x << "[m]" << pt.pose.position.y << "[m]" << angle << "[rad]" << std::endl;
        pose_pub.publish(pt);
        scan_flag = false;
        angle_flag = false;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void TargetHumanPoint::angle_callback(const std_msgs::Float32ConstPtr& msg)
{
  std_msgs::Float32 _angle = *msg;
  angle = _angle.data / 180.0 * M_PI;
  angle_flag = true;
}

void TargetHumanPoint::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan = *msg;
  scan_flag = true;
}

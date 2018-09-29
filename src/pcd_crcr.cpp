#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCDCrcrer
{
public:
  PCDCrcrer(void);
  void process(void);
  void set_output_file_name(std::string);
  void pcd_callback(const sensor_msgs::PointCloud2ConstPtr&);
  void crcr_callback(const geometry_msgs::PoseStampedConstPtr&);

private:
  void crcr(void);

  ros::NodeHandle nh;
  ros::Publisher pcd_pub;
  ros::Publisher crcr_point_pub;
  ros::Publisher crcr_line_pub;
  ros::Subscriber pcd_sub;
  ros::Subscriber crcr_sub;
  std::string output_file_name;
  sensor_msgs::PointCloud2 pcd_map;
  visualization_msgs::Marker crcr_points;
  visualization_msgs::Marker crcr_lines;
  bool crcr_flag;
  double x_upper_limit;
  double y_upper_limit;
  double x_lower_limit;
  double y_lower_limit;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_crcr");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  std::string OUTPUT_FILE_NAME;
  local_nh.getParam("OUTPUT_FILE_NAME", OUTPUT_FILE_NAME);

  PCDCrcrer crcr;
  crcr.set_output_file_name(OUTPUT_FILE_NAME);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    crcr.process();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

PCDCrcrer::PCDCrcrer(void)
{
  pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/crcred_map", 1);
  pcd_sub = nh.subscribe("/cloud_pcd", 1, &PCDCrcrer::pcd_callback, this);
  crcr_point_pub = nh.advertise<visualization_msgs::Marker>("/crcr/points", 1);
  crcr_line_pub = nh.advertise<visualization_msgs::Marker>("/crcr/lines", 1);
  crcr_sub = nh.subscribe("/move_base_simple/goal", 1, &PCDCrcrer::crcr_callback, this);
  crcr_flag = false;
  crcr_points.header.frame_id = "map";
  crcr_points.ns = "crcr_points";
  crcr_points.type = visualization_msgs::Marker::POINTS;
  crcr_points.action = visualization_msgs::Marker::ADD;
  crcr_points.color.r = 1;
  crcr_points.color.g = 0;
  crcr_points.color.b = 0;
  crcr_points.color.a = 1;
  crcr_points.scale.x = 0.3;
  crcr_points.scale.y = 0.3;
  crcr_points.scale.z = 0.01;
  crcr_points.lifetime = ros::Duration(0);
  crcr_lines.header.frame_id = "map";
  crcr_lines.ns = "crcr_lines";
  crcr_lines.type = visualization_msgs::Marker::LINE_LIST;
  crcr_lines.action = visualization_msgs::Marker::ADD;
  crcr_lines.color.r = 0;
  crcr_lines.color.g = 0;
  crcr_lines.color.b = 1;
  crcr_lines.color.a = 1;
  crcr_lines.scale.x = 0.3;
  crcr_lines.lifetime = ros::Duration(0);
}

void PCDCrcrer::process(void)
{
  pcd_pub.publish(pcd_map);
  crcr_point_pub.publish(crcr_points);
  crcr_line_pub.publish(crcr_lines);
  if(crcr_flag){
    crcr();
  }
}

void PCDCrcrer::set_output_file_name(std::string str)
{
  output_file_name = str;
  //std::cout << output_file_name << std::endl;
}

void PCDCrcrer::pcd_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  sensor_msgs::PointCloud2 temp = *msg;
  pcd_map = temp;
}

void PCDCrcrer::crcr_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped crcr_p = *msg;
  crcr_points.points.push_back(crcr_p.pose.position);
  if(crcr_points.points.size() == 4){
    crcr_flag = true;
    x_upper_limit = crcr_points.points[0].x;
    x_lower_limit = crcr_points.points[0].x;
    y_upper_limit = crcr_points.points[0].y;
    y_lower_limit = crcr_points.points[0].y;
    for(int i=1;i<4;i++){
      if(x_upper_limit < crcr_points.points[i].x){
        x_upper_limit = crcr_points.points[i].x;
      }else if(x_lower_limit > crcr_points.points[i].x){
        x_lower_limit = crcr_points.points[i].x;
      }
      if(y_upper_limit < crcr_points.points[i].y){
        y_upper_limit = crcr_points.points[i].y;
      }else if(y_lower_limit > crcr_points.points[i].y){
        y_lower_limit = crcr_points.points[i].y;
      }
    }
    geometry_msgs::Point temp;
    temp.x = x_upper_limit;
    temp.y = y_upper_limit;
    crcr_lines.points.push_back(temp);
    temp.x = x_lower_limit;
    temp.y = y_upper_limit;
    crcr_lines.points.push_back(temp);
    crcr_lines.points.push_back(temp);
    temp.x = x_lower_limit;
    temp.y = y_lower_limit;
    crcr_lines.points.push_back(temp);
    crcr_lines.points.push_back(temp);
    temp.x = x_upper_limit;
    temp.y = y_lower_limit;
    crcr_lines.points.push_back(temp);
    crcr_lines.points.push_back(temp);
    crcr_lines.points.push_back(crcr_lines.points[0]);
  }
}

void PCDCrcrer::crcr(void)
{

  pcl::io::savePCDFile(output_file_name, pcd_map);
  crcr_flag = false;
  crcr_points.points.clear();
  crcr_lines.points.clear();
}

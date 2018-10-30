#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

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
  int crcr_count;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
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
  crcr_count = 0;
  cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
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
  pcl::fromROSMsg(pcd_map, *cloud_ptr);
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
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Sampling cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i=0;i<(*cloud_ptr).size();i++){
    pcl::PointXYZ pt(cloud_ptr->points[i].x, cloud_ptr->points[i].y, cloud_ptr->points[i].z);
    if(((pt.x < x_upper_limit) && (pt.x > x_lower_limit) && (pt.y < y_upper_limit) && (pt.y > y_lower_limit)) && (pt.z < 2.0)){
      inliers->indices.push_back(i);
      _cloud_ptr->points.push_back(pt);
    }
  }
  extract.setInputCloud(cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_ptr);
  // Surface segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(_cloud_ptr);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1);
  seg.setMaxIterations(100);
  seg.setProbability(0.95);
  seg.segment(*inliers, *coefficients);
  extract.setInputCloud(_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*_cloud_ptr);

  *cloud_ptr += *_cloud_ptr;

  pcl::toROSMsg(*cloud_ptr, pcd_map);
  std::string fname = output_file_name + "_" + std::to_string(crcr_count) + ".pcd";
  pcl::io::savePCDFile(fname, pcd_map);
  std::cout << "Saved as " << fname << std::endl;
  crcr_flag = false;
  crcr_points.points.clear();
  crcr_lines.points.clear();
  crcr_count++;
}


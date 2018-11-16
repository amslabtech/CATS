#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

sensor_msgs::Imu imu_data;
sensor_msgs::Imu imu_data_offset;

int IMU_COUNT = 10000;
int imu_count = 0;

double yawrate_ = 0;
double offset_yawrate = 0;
bool received_flag = false;
// double OFFSET_YAWRATE = 0.00206676;
double OFFSET_YAWRATE = 0.001944;
ros::Time first_time;
bool first_flag = false;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  imu_data = *msg;
  if(!first_flag){
	first_time = msg->header.stamp;
	first_flag = true;
  }
	  
  // if(imu_count < IMU_COUNT){
   if((imu_data.header.stamp - first_time) < ros::Duration(15.0)){
	yawrate_ += imu_data.angular_velocity.z;
	std::cout << "=== calibrating ===" << std::endl;
	std::cout << imu_count << " / " << IMU_COUNT << std::endl;
	imu_count++;
   }
   else{
	offset_yawrate = yawrate_ / (double)IMU_COUNT;
	std::cout << "=== yawrate ===" << std::endl;
	std::cout << offset_yawrate << "[rad/s]" << std::endl;;
	std::cout << imu_data.angular_velocity.z << "[rad/s]" << std::endl;
	std::cout << imu_data.angular_velocity.z - offset_yawrate << "[rad/s]" << std::endl;
	received_flag = true;
   }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tkhsh_imu");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("IMU_COUNT", IMU_COUNT);

  ros::Subscriber imu_sub = nh.subscribe("/imu/data", 100, imu_callback);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data/calibrated", 100);

  ros::Rate loop_rate(50);

  while(ros::ok()){
	if(received_flag){
	  // imu_data.angular_velocity.z -= OFFSET_YAWRATE;
	  imu_data.angular_velocity.z -= offset_yawrate;
	  imu_pub.publish(imu_data);
	  received_flag = false;
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

using namespace std;

#define GPS_change 100

class Map2Gps{
	private:
		ros::NodeHandle n;
		ros::Subscriber change_sub;
		ros::Subscriber map_matching_sub;
		ros::Subscriber gps_sub;

		tf::TransformBroadcaster br;
		tf::Transform transform;

		nav_msgs::Odometry matching;
		nav_msgs::Odometry gps;

		bool flag;
		bool start_flag;

	public:
		Map2Gps(ros::NodeHandle n);
		void map_matchingCallback(const nav_msgs::Odometry msg);
		void gpsCallback(const nav_msgs::Odometry msg);
		void changeCallback(const std_msgs::BoolConstPtr input);//mapの切り替え

		void tf_broad(nav_msgs::Odometry msg);
		void process();

};

Map2Gps::Map2Gps(ros::NodeHandle n) :
	flag(false),start_flag(false)
{
	change_sub = n.subscribe("/ndt2gps", 100, &Map2Gps::changeCallback, this);
	map_matching_sub = n.subscribe("/lcl_ekf", 100, &Map2Gps::map_matchingCallback, this);
	gps_sub = n.subscribe("/gps_odom_nofix_tkb", 100, &Map2Gps::gpsCallback, this);

}


int main (int argc, char** argv){
	ros::init(argc,argv,"tf_map2gps");
	ros::NodeHandle n;

	Map2Gps map2gps(n);
	ros::Rate loop_rate(100);

	cout <<"----- tf_pub ok ------" <<endl;

	while(ros::ok()){
		map2gps.process();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}



void 
Map2Gps::changeCallback(const std_msgs::BoolConstPtr input){
	if(input->data) flag =true;
	else flag = false;
}


void 
Map2Gps::map_matchingCallback(const nav_msgs::Odometry msg){
	matching = msg;
	start_flag = true;
}


void 
Map2Gps::gpsCallback(const nav_msgs::Odometry msg){
	gps = msg;
	start_flag = true;
}

void 
Map2Gps::tf_broad(nav_msgs::Odometry msg){


	transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg.pose.pose.orientation.z);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, msg.header.stamp , "/map", "/matching_base_link"));

}

void
Map2Gps::process(){
	if(start_flag){
		if(flag) {
			cout<<"gps now"<<endl;
			tf_broad(gps);
		}
		else {
			cout<<"map matching now"<<endl;
			tf_broad(matching);
		}
	}

}

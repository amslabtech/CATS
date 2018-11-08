#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>

using namespace std;

class Map2Gps{
	private:
		ros::NodeHandle n;
		ros::Subscriber wp_sub;
		ros::Subscriber map_matching_sub;
		ros::Subscriber gps_sub;

		tf::TransformBroadcaster br;
		tf::Transform transform;

		nav_msgs::Odometry matching;
		nav_msgs::Odometry gps;

		bool flag;

	public:
		Map2Gps(ros::NodeHandle n);
		void map_matchingCallback(const nav_msgs::Odometry msg);
		void gpsCallback(const nav_msgs::Odometry msg);
		void wpCallback(const std_msgs::Int32ConstPtr input);//mapの切り替え

		void tf_broad(nav_msgs::Odometry msg);
		void process();

};

Map2Gps::Map2Gps(ros::NodeHandle n) :
	flag(true)
{
	wp_sub = n.subscribe("/waypoint/now", 100, &Map2Gps::wpCallback, this);
	map_matching_sub = n.subscribe("/lcl_ekf", 100, &Map2Gps::map_matchingCallback, this);
	gps_sub = n.subscribe("/lcl_gps", 100, &Map2Gps::gpsCallback, this);

}


int main (int argc, char** argv){
	ros::init(argc,argv,"tf_map2gps");
	ros::NodeHandle n;

	Map2Gps map2gps(n);
	ros::Rate loop_rate(100);

	while(ros::ok()){
		map2gps.process();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}



void 
Map2Gps::wpCallback(const std_msgs::Int32ConstPtr input){
	if(input->data < 32) flag =true;
	else flag = false;
}


void 
Map2Gps::map_matchingCallback(const nav_msgs::Odometry msg){
	matching = msg;	
}


void 
Map2Gps::gpsCallback(const nav_msgs::Odometry msg){
	gps = msg;
}

void 
Map2Gps::tf_broad(nav_msgs::Odometry msg){


	transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now() , "/map", "/matching_base_link"));

}

void
Map2Gps::process(){
	if(flag) tf_broad(matching);
	else tf_broad(gps);

}

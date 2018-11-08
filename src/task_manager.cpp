#include <ros/ros.h>
#include <std_msgs/Int32.h>

std_msgs::Int32 wp;

// イベントのあるwp_id
const int t_l_1 = 2;
bool flag0 = true;
const int kill_t_l_1 = 3;
bool flag1 = true;
const int t_l_2 = 5;
bool flag2 = true;

// タスク番号
const int TRAFFIC_LIGHT = 1;
const int HUMAN_RECOGNITION = 2;
const int KILL_DARKNET = 3;

void wp_callback(const std_msgs::Int32ConstPtr& msg)
{
  wp = *msg;
  std::cout << "next id : " << wp.data << std::endl;
}

void process(void)
{
  std::cout << "=== task_manager ===" << std::endl;

  ros::NodeHandle nh;

  ros::Subscriber wp_sub = nh.subscribe("/waypoint/now", 10, wp_callback);
  ros::Publisher task_pub = nh.advertise<std_msgs::Int32>("/task", 10);

  ros::Rate r(10);

  while(ros::ok()){
    if(wp.data == t_l_1){
	  if(flag0){
		std_msgs::Int32 t;
		t.data = TRAFFIC_LIGHT;
		task_pub.publish(t);
		flag0 = false;
	  }
	}else if(wp.data == kill_t_l_1){
	  if(flag1){
        std_msgs::Int32 t;
        t.data = KILL_DARKNET;
        task_pub.publish(t);
		flag1 = false;
      }
	}else if(wp.data == t_l_2){
	  if(flag2){
		std_msgs::Int32 t;
		t.data = TRAFFIC_LIGHT;
		task_pub.publish(t);
		flag2 = false;
	  }
	}

  	ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");
  process();
}

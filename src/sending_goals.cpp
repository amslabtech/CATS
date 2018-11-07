#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float goals[7][7] =  {{16.711, -0.364, 0, 0, 0, 0.693, 0.722},
					  {16.999, 6.716, 0, 0, 0, 1.0, -0.005},
					  {-16.529, 8.198, 0, 0, 0, -0.719, 0.695},
					  {-17.503, -6.118, 0, 0, 0, -0.019, 1.0},
					  {16.477, -6.905, 0, 0, 0, -0.019, 1.0},
					  {16.794, -0.804, 0, 0, 0, 0.999, 0.033},
					  {0.516, 0.112, 0, 0, 0, 0.996, -0.091}};

int main(int argc, char** argv){
  ros::init(argc, argv, "sending_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  int i = 0;

  while(ros::ok()){
	goal.target_pose.pose.position.x = goals[i][0];
	goal.target_pose.pose.position.y = goals[i][1];
	goal.target_pose.pose.position.z = goals[i][2];
	goal.target_pose.pose.orientation.x = goals[i][3];
	goal.target_pose.pose.orientation.y = goals[i][4];
	goal.target_pose.pose.orientation.z = goals[i][5];
	goal.target_pose.pose.orientation.w = goals[i][6];
	if (i==7) break;
	  ROS_INFO("Sending goal: No.%d", i+1);
	  ac.sendGoal(goal);
	  bool succeeded = ac.waitForResult(ros::Duration(120.0));
	  actionlib::SimpleClientGoalState state = ac.getState();
	  if(succeeded) {
		ROS_INFO("Succeeded: No.%d (%s)",i+1, state.toString().c_str());
	  }else {
		ROS_INFO("Failed: No.%d (%s)",i+1, state.toString().c_str());
	  }
      i++;
    }
  return 0;
}


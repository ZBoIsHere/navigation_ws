#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <csc_nav2d_navigator/MoveToPosition2DAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <csc_nav2d_navigator/commands.h>

using namespace std;

typedef actionlib::SimpleActionClient<csc_nav2d_navigator::MoveToPosition2DAction> MoveClient;
MoveClient* gMoveClient;
ros::Subscriber pose_Subscriber;
csc_nav2d_navigator::MoveToPosition2DGoal goal;
struct pose
{
  double x, y, z;
  double pitch, roll, yaw;
};
pose robot_currnet_pose__;
pose goal_pose__;



/**
 * @brief receiveGoal
 * 接收来自rviz的目标信息，转发goal到server，不管server的返回信息。
 * 未来可将这个部分替换掉，自动进行目标点发送
 * @param msg
 */
void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(msg->pose.position.x == -10000 && msg->pose.position.y == -10000)
		return;
	 goal.target_pose.x = msg->pose.position.x;
	 goal.target_pose.y = msg->pose.position.y;
	 goal.target_pose.theta = tf::getYaw(msg->pose.orientation);

	//goal.target_pose.x = goal_point[0][0];
	//goal.target_pose.y = goal_point[0][1];
	//goal.target_pose.theta = goal_point[0][2];
	goal.target_distance = 0.1;  //目标点的容忍距离
	goal.target_angle = 0.1;  //目标点的容忍角度 rad
	
	gMoveClient->sendGoal(goal);
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "SetGoal");
	ros::NodeHandle n;
	ros::Subscriber goalSubscriber = n.subscribe("/move_base_simple/goal", 1, &receiveGoal);//原始是goal，但是rviz发布的是/move_base_simple/goa
	
	gMoveClient = new MoveClient(NAV_MOVE_ACTION, true);
	gMoveClient->waitForServer();

	ros::spin();
		
	delete gMoveClient;
	return 0;
}

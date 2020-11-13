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
ros::Publisher  arrive_charge_area;
struct pose
{
  double x, y, z;
  double pitch, roll, yaw;
};
pose robot_currnet_pose__;
pose goal_pose__;

bool going_charge = false;
bool complete_charge = false;
//bool first_param_goal=true;
//   charge station point   ========= other point=========
float goal_point[ ][3]={1.05313, 0.232326, 1.99338 , 2.5383 ,-3.20821 , 1.93924};  //人为设定目标点
int goal_i = 0;//目标点索引
int robot_message;


/**
 * @brief receiveGoal
 * 接收来自rviz的目标信息，转发goal到server，不管server的返回信息。
 * 未来可将这个部分替换掉，自动进行目标点发送
 * @param msg
 */
void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// 	goal.target_pose.x = msg->pose.position.x;
	// goal.target_pose.y = msg->pose.position.y;
	// goal.target_pose.theta = tf::getYaw(msg->pose.orientation);

	goal.target_pose.x = goal_point[0][0];
	goal.target_pose.y = goal_point[0][1];
	goal.target_pose.theta = goal_point[0][2];
	goal.target_distance = 0.1;  //目标点的容忍距离
	goal.target_angle = 0.1;  //目标点的容忍角度 rad
	
	gMoveClient->sendGoal(goal);
	
}
bool if_receive = true;
void ndt_pose_Callback(geometry_msgs::PoseStampedConstPtr msg)
{
	if (!if_receive){
		return;
	}
	robot_currnet_pose__.x = msg->pose.position.x;
	robot_currnet_pose__.y = msg->pose.position.y;
	geometry_msgs::Quaternion orientation = msg->pose.orientation;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
	robot_currnet_pose__.yaw = yaw;



	//std::cout<<"the ndt localization is (x y yaw):"<<robot_currnet_pose__.x<<" "<< robot_currnet_pose__.y<<" "<<robot_currnet_pose__.yaw<<endl;
	// if(sqrt(pow(robot_currnet_pose__.x-goal_point[goal_i][0],2)+pow(robot_currnet_pose__.y-goal_point[goal_i][1],2) ) < 0.3 
	// 	  														 && abs(robot_currnet_pose__.yaw - goal_point[goal_i][2]) < 0.5 )
	if(sqrt(pow(robot_currnet_pose__.x-goal_point[goal_i][0],2)+pow(robot_currnet_pose__.y-goal_point[goal_i][1],2) ) < 0.3 && robot_currnet_pose__.yaw - goal_point[goal_i][2] < 0.4 )
	{
		cout<<"Robot has arrived "<< goal_i <<" point area !!!"<<endl;
		goal_i++;
		if(goal_i>1) //2 ge 目标点循环
			goal_i=0;
		goal.target_pose.x = goal_point[goal_i][0];
		goal.target_pose.y = goal_point[goal_i][1];
		goal.target_pose.theta = goal_point[goal_i][2];
		goal.target_distance = 0.1;  //目标点的容忍距离
		goal.target_angle = 0.1;  //目标点的容忍角度 rad
		gMoveClient->sendGoal(goal);
	}

	// else  // go charge
	// {
		// cout<<"Robot going charge !!!"<<endl;
		// goal.target_pose.x = 1.25549; //charge point
		// goal.target_pose.y = 0.0160733;
		// goal.target_pose.theta = 2.02087;
		// goal.target_distance = 0.3;  //目标点的容忍距离
		// goal.target_angle = 0.1;  //目标点的容忍角度 rad
		// gMoveClient->sendGoal(goal);
		// cout << "sqrt::" << sqrt(pow(robot_currnet_pose__.x-goal_point[0][0],2)+pow(robot_currnet_pose__.y-goal_point[0][1],2) ) << endl;

		// arrived charge station point
		if(sqrt(pow(robot_currnet_pose__.x-goal_point[0][0],2)+pow(robot_currnet_pose__.y-goal_point[0][1],2) ) < 0.3 && robot_currnet_pose__.yaw - goal_point[0][2] < 0.4)
		{
			std_msgs::String temp;
			temp.data = "yes";
			arrive_charge_area.publish(temp);
			cout<<"msg    is    yesyesyesyesyes"<<endl;
			// going_charge = false;
			
			// goal.target_pose.x = goal_point[goal_i][0];
			// goal.target_pose.y = goal_point[goal_i][1];
			// goal.target_pose.theta = goal_point[goal_i][2];
			// goal.target_distance = 0.3;  //目标点的容忍距离
			// goal.target_angle = 0.1;  //目标点的容忍角度 rad
			// gMoveClient->sendGoal(goal);
		}
		if(sqrt(pow(robot_currnet_pose__.x-goal_point[0][0],2)+pow(robot_currnet_pose__.y-goal_point[0][1],2) ) > 2)
		{
			std_msgs::String temp;
			temp.data = "no";
			arrive_charge_area.publish(temp);
			//cout<<"msg    is    nonononono"<<endl;
		}

	// }

}

void robot_message_Callback(std_msgs::Int32Ptr msg)
{
	robot_message = msg->data;
	if(robot_message==1)
	{
		going_charge = true;
		robot_message = 0;
	}
	else if(robot_message==2 )
	{
		complete_charge = true;
		robot_message = 0;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SetGoal");
	ros::NodeHandle n;
  arrive_charge_area = n.advertise<std_msgs::String>("/if_arrive_charge", 10);
	ros::Subscriber goalSubscriber = n.subscribe("/move_base_simple/goal", 1, &receiveGoal);//原始是goal，但是rviz发布的是/move_base_simple/goa
	pose_Subscriber = n.subscribe("/ndt/current_pose", 10, &ndt_pose_Callback);
	ros::Subscriber robot_message_sub = n.subscribe("udpThread", 10, &robot_message_Callback);
	
	gMoveClient = new MoveClient(NAV_MOVE_ACTION, true);
	gMoveClient->waitForServer();

	ros::spin();
		
	delete gMoveClient;
	return 0;
}

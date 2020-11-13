#include <ros/ros.h>

#include <csc_nav2d_navigator/RobotNavigator.h>
using namespace ros;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	RobotNavigator robNav;
	
  ros::spin();


	return 0;
}

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <csc_nav2d_operator/RobotOperator.h>

using namespace ros;
/**
 * @brief main
 * 订阅自定义的速度、转向、模式命令，使用不同的策略。
 * 接收到的速度和转向信息经过限幅直接转发成geometry_msgs::Twist类型的数据。送给驱动层。
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;

	RobotOperator robOp;
	
	Rate loopRate(10);
	while(ok())
	{
		robOp.executeCommand();
		spinOnce();
		loopRate.sleep();
	}
	return 0;	
}

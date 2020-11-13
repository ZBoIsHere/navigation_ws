#ifndef OPERATOR_H
#define OPERATOR_H

#define NODE_NAME     "operator"
#define COMMAND_TOPIC "cmd"
#define CONTROL_TOPIC "cmd_vel"


#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <csc_nav2d_operator/cmd.h>

#include <string>

class RobotOperator
{
public:
	// Default Constructor & Destructor
	RobotOperator();
	~RobotOperator();
	
	// Public Methods
	/**
	 * @brief Callback function to receive move commands
	 * @param msg Command-Message
	 * Direction [-1.0 .. 1.0]: -1(rotate left); 0(straight); 1(rotate right)
	 * Velocity  [-1.0 .. 1.0]: -1(full speed back); 0(stop); 1(full speed ahead)
	 * Mode: 0(Avoid obstacles); 1(Stop at obstacles)
	 */
  void receiveCommand(const csc_nav2d_operator::cmd::ConstPtr& msg);

	/**
	 * @brief Generates and sends Twist-Message to Robot
	 * This is the Operator's core function and should be called periodically
	 */
	void executeCommand();

private:
	
	tf::TransformListener mTfListener;
	
	ros::Subscriber mCommandSubscriber;
	ros::Publisher mControlPublisher;
	
	double mDesiredVelocity;
	double mDesiredDirection;
	double mCurrentVelocity;
	double mCurrentDirection;
	int mDriveMode;
	double mMaxVelocity;
    double mMaxAngleVelocity;
};

#endif

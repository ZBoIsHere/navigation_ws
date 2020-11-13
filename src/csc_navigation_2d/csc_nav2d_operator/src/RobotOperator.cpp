#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>

#include <csc_nav2d_operator/RobotOperator.h>

#define PI 3.14159265
/**
 * @brief RobotOperator::RobotOperator
 * 订阅自定义的速度、转向、模式命令，使用不同的策略。
 * 这里已经改为“直通模式”。
 * 接收到的速度和转向信息经过限幅直接转发成geometry_msgs::Twist类型的数据。送给驱动层。
 */
RobotOperator::RobotOperator()
{
    ros::NodeHandle robotNode;
    mCommandSubscriber = robotNode.subscribe(COMMAND_TOPIC, 1, &RobotOperator::receiveCommand, this);
    mControlPublisher = robotNode.advertise<geometry_msgs::Twist>(CONTROL_TOPIC, 10);

    // Get parameters from the parameter server
    ros::NodeHandle operatorNode("~/");
    operatorNode.param("max_velocity", mMaxVelocity, 1.0);
    operatorNode.param("max_angle_velocity", mMaxAngleVelocity, 1.0);

    // Set internal parameters
    mDesiredDirection = 0;
    mDesiredVelocity = 0;
    mCurrentDirection = 0;
    mCurrentVelocity = 0;
    mDriveMode = 0;
}

RobotOperator::~RobotOperator()
{

}

/**
 * @brief RobotOperator::receiveCommand 接收自定义的速度、转向、模式命令
 * @param msg
 */
void RobotOperator::receiveCommand(const csc_nav2d_operator::cmd::ConstPtr& msg)
{
    mDesiredDirection = msg->Turn;
    mDesiredVelocity = msg->Velocity;
    mDriveMode = msg->Mode;
}
/**
 * @brief RobotOperator::executeCommand 定时执行命令
 */
void RobotOperator::executeCommand()
{
    switch(mDriveMode)
    {
    case 0://pause mode
        mCurrentDirection = mDesiredDirection;
        mCurrentVelocity = mDesiredVelocity;
        break;
    case 1:
        mCurrentDirection = mDesiredDirection;
        mCurrentVelocity = mDesiredVelocity;
        break;
    case 2:
        mCurrentDirection = mDesiredDirection;
        mCurrentVelocity = mDesiredVelocity;
        break;

    default:
        ROS_ERROR("Invalid drive mode!");
        mCurrentVelocity = 0.0;
    }

    // Publish result via Twist-Message
    geometry_msgs::Twist controlMsg;

    controlMsg.linear.x = mMaxVelocity * mCurrentVelocity;
    controlMsg.angular.z = mMaxAngleVelocity * mCurrentDirection;//不考虑后退策略

    mControlPublisher.publish(controlMsg);
}

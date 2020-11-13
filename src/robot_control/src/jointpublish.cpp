#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

using namespace std;

int main(int argc, char** argv)
{
    /*
    新建一个JointState的 publisher 发送十二个关节的信息；
    // Message Type: sensor_msgs::JointState
    // Header header
    // String[] name
    // float64[] position
    // float64[] velocity
    // float64[] effort
    // 分别代表位置（这里位置指定的是角度） 速度 受力；
    */
    ros::init(argc, argv, "jointpublish");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(10);

    int i=0;

    const double degree = M_PI/180;
    double joint_angle[12] = {0};
    sensor_msgs::JointState joint_state;


    /*
    joint_state 以一个12元素的array组成，分别指代狗的十二个关节
    分为四条腿；每个腿上从上到下指定三个关节
    // 命名规则：
    // Left Front -- LF
    // Left Behind -- LB
    // Right Front -- RF
    // Right Behind -- RB
    */
    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(12);
        joint_state.position.resize(12);

        joint_state.name[0]="LF_Joint";
        joint_state.position[0] = joint_angle[0];
                
        joint_state.name[1]="LF_Joint_1";
        joint_state.position[1] = joint_angle[1];
            
        joint_state.name[2]="LF_Joint_2";
        joint_state.position[2] = joint_angle[2];
                
        joint_state.name[3]="LB_Joint";
        joint_state.position[3] = joint_angle[3];

        joint_state.name[4]="LB_Joint_1";
        joint_state.position[4] = joint_angle[4];

        joint_state.name[5]="LB_Joint_2";
        joint_state.position[5] = joint_angle[5];

        joint_state.name[6]="RF_Joint";
        joint_state.position[6] = joint_angle[6];

        joint_state.name[7]="RF_Joint_1";
        joint_state.position[7] = joint_angle[7];

        joint_state.name[8]="RF_Joint_2";
        joint_state.position[8] = joint_angle[8];

        joint_state.name[9]="RB_Joint";
        joint_state.position[9] = joint_angle[9];

        joint_state.name[10]="RB_Joint_1";
        joint_state.position[10] = joint_angle[10];

        joint_state.name[11]="RB_Joint_2";
        joint_state.position[11] = joint_angle[11];
        //send the joint state and transform

        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

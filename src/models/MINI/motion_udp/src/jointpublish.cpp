#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

// #include <motion_udp/ContactData.h>
#include <motion_udp/ImuData.h>
#include <motion_udp/JointData.h>
#include <motion_udp/JointType.h>
#include <motion_udp/MessageData.h>
#include <motion_udp/RobotJointData.h>
#include <motion_udp/udpMessage.h>

using namespace std;

sensor_msgs::JointState joint_state;
ros::Publisher joint_pub;

void rev_udpmsg(const motion_udp::udpMessage::ConstPtr& msg)
{
  joint_state.header.stamp = ros::Time::now();//s//msg->message_data.timestamp
  //cout<<"time:"<<ros::Time::now();

  joint_state.name.resize(12);
  joint_state.position.resize(12);

  joint_state.name[0]="LF_Joint";
  joint_state.position[0] = -msg->message_data.robot_joint_data.position.fl.hip_x;

  joint_state.name[1]="LF_Joint_1";
  joint_state.position[1] = -msg->message_data.robot_joint_data.position.fl.hip_y;

  joint_state.name[2]="LF_Joint_2";
  joint_state.position[2] = -msg->message_data.robot_joint_data.position.fl.knee;

  joint_state.name[3]="LB_Joint";
  joint_state.position[3] = -msg->message_data.robot_joint_data.position.hl.hip_x;

  joint_state.name[4]="LB_Joint_1";
  joint_state.position[4] = -msg->message_data.robot_joint_data.position.hl.hip_y;

  joint_state.name[5]="LB_Joint_2";
  joint_state.position[5] = -msg->message_data.robot_joint_data.position.hl.knee;

  joint_state.name[6]="RF_Joint";
  joint_state.position[6] = -msg->message_data.robot_joint_data.position.fr.hip_x;

  joint_state.name[7]="RF_Joint_1";
  joint_state.position[7] = -msg->message_data.robot_joint_data.position.fr.hip_y;

  joint_state.name[8]="RF_Joint_2";
  joint_state.position[8] = -msg->message_data.robot_joint_data.position.fr.knee;

  joint_state.name[9]="RB_Joint";
  joint_state.position[9] = -msg->message_data.robot_joint_data.position.hr.hip_x;

  joint_state.name[10]="RB_Joint_1";
  joint_state.position[10] = -msg->message_data.robot_joint_data.position.hr.hip_y;

  joint_state.name[11]="RB_Joint_2";
  joint_state.position[11] = -msg->message_data.robot_joint_data.position.hr.knee;

  joint_pub.publish(joint_state);
}
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

    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Subscriber udp_sub = n.subscribe("udp_msg",10,rev_udpmsg);

    //ros::Rate loop_rate(100);

    int i=0;

    const double degree = M_PI/180;
    double joint_angle[12] = {0};

    /*
    joint_state 以一个12元素的array组成，分别指代狗的十二个关节
    分为四条腿；每个腿上从上到下指定三个关节
    // 命名规则：
    // Left Front -- LF
    // Left Behind -- LB
    // Right Front -- RF
    // Right Behind -- RB
    */
//    while (ros::ok()) {
//        joint_state.header.stamp = ros::Time::now();
//        joint_state.name.resize(12);
//        joint_state.position.resize(12);

//        joint_state.name[0]="LF_Joint";
//        joint_state.position[0] = joint_angle[0];
                
//        joint_state.name[1]="LF_Joint_1";
//        joint_state.position[1] = joint_angle[1];
            
//        joint_state.name[2]="LF_Joint_2";
//        joint_state.position[2] = joint_angle[2];
                
//        joint_state.name[3]="LB_Joint";
//        joint_state.position[3] = joint_angle[3];

//        joint_state.name[4]="LB_Joint_1";
//        joint_state.position[4] = joint_angle[4];

//        joint_state.name[5]="LB_Joint_2";
//        joint_state.position[5] = joint_angle[5];

//        joint_state.name[6]="RF_Joint";
//        joint_state.position[6] = joint_angle[6];

//        joint_state.name[7]="RF_Joint_1";
//        joint_state.position[7] = joint_angle[7];

//        joint_state.name[8]="RF_Joint_2";
//        joint_state.position[8] = joint_angle[8];

//        joint_state.name[9]="RB_Joint";
//        joint_state.position[9] = joint_angle[9];

//        joint_state.name[10]="RB_Joint_1";
//        joint_state.position[10] = joint_angle[10];

//        joint_state.name[11]="RB_Joint_2";
//        joint_state.position[11] = joint_angle[11];
        //send the joint state and transform

//        if(i<180)
//            i++;
//        else i=0;
//        for(int j=0;j<12;j++)
//        {
//            joint_angle[j]= i*degree;
//        }
//        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
//        loop_rate.sleep();
//    }
    ros::spin();

    return 0;
}

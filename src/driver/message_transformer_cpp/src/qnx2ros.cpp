#include <errno.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <netinet/in.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>

#include "moving_average.h"
#include "tf/transform_datatypes.h"

using namespace std;

#define SERV_PORT 43897
#define PI 3.1415926

#pragma pack(4)
struct RobotStateUpload {
  double rpy[3];
  double rpy_vel[3];
  double xyz_acc[3];
  double pos_world[3];
  double vel_world[3];
  unsigned touch_down_and_stair_trot;
  bool battery_percentage;
  unsigned error_state;
  int auto_charge;
  double battery_volt;
  double driver_temp;
  double motor_temp;
  int robot_basic_state;
  int robot_gait_state;
  int robot_task_state;
};
struct RobotStateReceived {
  int code;
  int size;
  int cons_code;
  struct RobotStateUpload data;
};

struct JointStateUpload {
  double LF_Joint;
  double LF_Joint_1;
  double LF_Joint_2;
  double LB_Joint;
  double LB_Joint_1;
  double LB_Joint_2;
  double RF_Joint;
  double RF_Joint_1;
  double RF_Joint_2;
  double RB_Joint;
  double RB_Joint_1;
  double RB_Joint_2;
};
struct JointStateReceived {
  int code;
  int size;
  int cons_code;
  struct JointStateUpload data;
};

int main(int argc, char **argv) {
  // 网络初始化
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    perror("socket");
    exit(1);
  }
  struct sockaddr_in addr_serv;
  int len;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  addr_serv.sin_family = AF_INET;                     //使用IPV4地址
  addr_serv.sin_port = htons(SERV_PORT);              //端口
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  len = sizeof(addr_serv);
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
  {
    perror("bind error:");
    exit(1);
  }
  int recv_num = -1;
  char recv_buf[500];
  struct sockaddr_in addr_client;

  // ROS 节点
  ros::init(argc, argv, "qnx2ros");
  ros::NodeHandle nh;

  int filter_size;
  nh.param<int>("filter_size", filter_size, 1);
  MovingAverage filter_vel_x(filter_size);
  MovingAverage filter_vel_y(filter_size);
  MovingAverage filter_vel_theta(filter_size);

  ros::Publisher leg_odom_pub;
  ros::Publisher joint_state_pub;
  leg_odom_pub = nh.advertise<nav_msgs::Odometry>("leg_odom", 1);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0,
                        (struct sockaddr *)&addr_client, (socklen_t *)&len);
    // 发布里程计数据
    if (recv_num == sizeof(RobotStateReceived)) {
      RobotStateReceived *dr = (RobotStateReceived *)(recv_buf);
      RobotStateUpload *robot_state = &dr->data;
      // ROS_INFO_STREAM("CODE: " << dr->code);
      if (dr->code == 2305) {
        nav_msgs::Odometry leg_odom_data;
        leg_odom_data.header.frame_id = "odom";
        leg_odom_data.child_frame_id = "base_link";
        // Position
        leg_odom_data.header.stamp = ros::Time::now();
        leg_odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_state->rpy[2] / 180 * PI);
        leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
        leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
        // Velocity
        double yaw = robot_state->rpy[2] / 180 * PI;
        filter_vel_x.in(robot_state->vel_world[0]);
        filter_vel_y.in(robot_state->vel_world[1]);
        filter_vel_theta.in(robot_state->rpy_vel[2]);
        leg_odom_data.twist.twist.linear.x =
            +filter_vel_x.out() * cos(yaw) + filter_vel_y.out() * sin(yaw);
        leg_odom_data.twist.twist.linear.y =
            -filter_vel_x.out() * sin(yaw) + filter_vel_y.out() * cos(yaw);
        leg_odom_data.twist.twist.angular.z = filter_vel_theta.out();
        leg_odom_pub.publish(leg_odom_data);
      }
    };

    // 发布关节角数据
    if ((recv_num == sizeof(JointStateReceived))) {
      JointStateReceived *dr = (JointStateReceived *)(recv_buf);
      JointStateUpload *joint_state = &dr->data;
      // ROS_INFO_STREAM("CODE: " << dr->code);
      if (dr->code == 2306) {
        sensor_msgs::JointState joint_state_data;
        joint_state_data.header.stamp = ros::Time::now();
        joint_state_data.name.resize(12);
        joint_state_data.position.resize(12);

        joint_state_data.name[0] = "LF_Joint";
        joint_state_data.position[0] = - joint_state->LF_Joint;
        joint_state_data.name[1] = "LF_Joint_1";
        joint_state_data.position[1] = - joint_state->LF_Joint_1;
        joint_state_data.name[2] = "LF_Joint_2";
        joint_state_data.position[2] = - joint_state->LF_Joint_2;
        joint_state_data.name[3] = "LB_Joint";
        joint_state_data.position[3] = - joint_state->LB_Joint;
        joint_state_data.name[4] = "LB_Joint_1";
        joint_state_data.position[4] = - joint_state->LB_Joint_1;
        joint_state_data.name[5] = "LB_Joint_2";
        joint_state_data.position[5] = - joint_state->LB_Joint_2;
        joint_state_data.name[6] = "RF_Joint";
        joint_state_data.position[6] = - joint_state->RF_Joint;
        joint_state_data.name[7] = "RF_Joint_1";
        joint_state_data.position[7] = - joint_state->RF_Joint_1;
        joint_state_data.name[8] = "RF_Joint_2";
        joint_state_data.position[8] = - joint_state->RF_Joint_2;
        joint_state_data.name[9] = "RB_Joint";
        joint_state_data.position[9] = - joint_state->RB_Joint;
        joint_state_data.name[10] = "RB_Joint_1";
        joint_state_data.position[10] = - joint_state->RB_Joint_1;
        joint_state_data.name[11] = "RB_Joint_2";
        joint_state_data.position[11] = - joint_state->RB_Joint_2;
        joint_state_pub.publish(joint_state_data);
      }
    };
    loop_rate.sleep();
  }

  close(sock_fd);
  return 0;
}
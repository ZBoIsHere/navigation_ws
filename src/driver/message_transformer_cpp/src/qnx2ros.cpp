#include <errno.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
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
geometry_msgs::Quaternion imu_data_yaw;
nav_msgs::Odometry leg_odom_data;
ros::Publisher leg_odom_pub;

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
};
struct DataReceived {
  int code;
  int size;
  int cons_code;
  struct RobotStateUpload data;
};

int main(int argc, char **argv) {
  /************** sock  init *************/
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
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) /* 绑定socket */
  {
    perror("bind error:");
    exit(1);
  }
  int recv_num = -1;
  char recv_buf[500];
  struct sockaddr_in addr_client;

  /*********ros::init***********/
  ros::init(argc, argv, "qnx2ros");
  ros::NodeHandle nh;
  leg_odom_pub = nh.advertise<nav_msgs::Odometry>("leg_odom", 5);

  int filter_size;
  nh.param<int>("filter_size", filter_size, 40);
  leg_odom_data.header.frame_id = "odom";
  leg_odom_data.child_frame_id = "base_link";

  MovingAverage filter_vel_x(filter_size);
  MovingAverage filter_vel_y(filter_size);
  MovingAverage filter_vel_theta(filter_size);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0,
                        (struct sockaddr *)&addr_client, (socklen_t *)&len);
    // if ((recv_num != sizeof(RobotStateUpload) + 12)) continue;
    cout << "recvfrom." << endl;
    DataReceived *dr = (DataReceived *)(recv_buf);
    RobotStateUpload *robot_state = &dr->data;
    if (dr->code != 0x901)
      continue;
    cout << "RobotStateUpload." << endl;
    imu_data_yaw = tf::createQuaternionMsgFromYaw(robot_state->rpy[2] / 180 * PI);

    // Position
    ros::Time current_time = ros::Time::now();
    leg_odom_data.header.stamp = current_time;
    leg_odom_data.pose.pose.orientation = imu_data_yaw;
    leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
    leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
    leg_odom_data.pose.pose.position.z = 0.0;

    // Velocity
    double theta = robot_state->rpy[2] / 180 * PI;
    filter_vel_x.in(robot_state->vel_world[0]);
    filter_vel_y.in(robot_state->vel_world[1]);
    filter_vel_theta.in(robot_state->rpy_vel[2]);
    leg_odom_data.twist.twist.linear.x =
        +filter_vel_x.out() * cos(theta) + filter_vel_y.out() * sin(theta);
    leg_odom_data.twist.twist.linear.y =
        -filter_vel_x.out() * sin(theta) + filter_vel_y.out() * cos(theta);
    leg_odom_data.twist.twist.angular.z = filter_vel_theta.out();

    leg_odom_pub.publish(leg_odom_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  close(sock_fd);
  return 0;
}
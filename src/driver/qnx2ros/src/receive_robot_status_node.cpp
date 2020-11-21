#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>

#include "moving_average/moving_average.h"
#include "tf/transform_datatypes.h"

using namespace std;

#define SERV_PORT 43897
#define PI 3.1415926
sensor_msgs::Imu imu_data;
geometry_msgs::Quaternion imu_data_yaw;
geometry_msgs::Quaternion imu_data_qua;

nav_msgs::Odometry leg_odom_data;
geometry_msgs::PoseWithCovarianceStamped leg_pose_data;
ros::Publisher imu_pub;
ros::Publisher leg_odom_pub;
ros::Publisher leg_pose_pub;
// ros::Publisher rtk_vel_pub;
/*****sensor_msgs::NavSatFix***********/
// sensor_msgs::NavSatFix rtk_fix;//!!!!!!!!!!!!!!!!!!
// ros::Publisher rtk_fix_pub;//!!!!!!!!!!!!!!!!!!!!

#pragma pack(4)
struct RobotStateUpload {
  double rpy[3];
  double rpy_vel[3];
  double xyz_acc[3];
  double pos_world[3];
  double vel_world[3];
  unsigned touch_down_and_stair_trot;
  bool battery_percentage;
  unsigned error_state;  //[0]low battery; [1]hight temp
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
  addr_serv.sin_addr.s_addr = htonl(
      INADDR_ANY);  //自动获取IP地址  /*
                    // INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到
                    //*/
  len = sizeof(addr_serv);
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) <
      0) /* 绑定socket */
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
  // ros::Subscriber lidar_sub = nh.subscribe("/velodyne_points", 10,
  // lidar_callback);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 5);
  leg_odom_pub = nh.advertise<nav_msgs::Odometry>("leg_odom", 5);
  leg_pose_pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("leg_pose", 5);
  tf::TransformBroadcaster odom_broadcaster;

  int filter_size;
  nh.param<int>("filter_size", filter_size, 40);
  imu_data.header.frame_id = "/imu";
  leg_odom_data.header.frame_id = "odom";
  leg_pose_data.header.frame_id = "odom";
  leg_odom_data.child_frame_id = "base_link";

  MovingAverage filter_vel_x(filter_size);
  MovingAverage filter_vel_y(filter_size);
  MovingAverage filter_vel_theta(filter_size);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    // std::cout<<"server waiting
    // ......................................."<<endl; clock_t start,end; start
    // = clock();
    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0,
                        (struct sockaddr *)&addr_client, (socklen_t *)&len);
    // if(recv_num < 0){
    //   perror("recvfrom error:");
    //   exit(1);
    // }
    // usrcout<<"server receive " <<recv_num<< " bytes: "<<endl;
    // std::cout<<"sizeof(RobotStateUpload) " <<sizeof(RobotStateUpload)<< "
    // bytes: "<<endl;
    if ((recv_num != sizeof(RobotStateUpload) + 12)) continue;
    DataReceived *dr = (DataReceived *)(recv_buf);
    RobotStateUpload *robot_state = &dr->data;
    // std::cout<<"code: "<<dr->code<<endl;
    // std::cout<<"size: "<<dr->size<<endl;
    // std::cout<<"cons_code: "<<dr->cons_code<<endl;
    if (dr->code != 0x901) continue;
    // std::cout<<"imu rpy: "<<robot_state->rpy[0]<<" "<<robot_state->rpy[1]<<"
    // "<<robot_state->rpy[2]<<endl; std::cout<<"imu rpy_vel:
    // "<<robot_state->rpy_vel[0]<<" "<<robot_state->rpy_vel[1]<<"
    // "<<robot_state->rpy_vel[2]<<endl; std::cout<<"imu xyz_acc:
    // "<<robot_state->xyz_acc[0]<<" "<<robot_state->xyz_acc[1]<<"
    // "<<robot_state->xyz_acc[2]<<endl; std::cout<<"leg_odom_pos_world:
    // "<<robot_state->pos_world[0]<<" "<<robot_state->pos_world[1]<<"
    // "<<robot_state->pos_world[2]<<endl;

    imu_data_yaw = tf::createQuaternionMsgFromYaw(robot_state->rpy[2] / 180 *
                                                  PI);  // RPY to quaternion
    imu_data_qua = tf::createQuaternionMsgFromRollPitchYaw(
        robot_state->rpy[0] / 180 * PI, robot_state->rpy[1] / 180 * PI,
        robot_state->rpy[2] / 180 * PI);  // RPY to quaternion
    /******imu_data*******/
    ros::Time current_time = ros::Time::now();
    imu_data.header.stamp = current_time;
    imu_data.orientation = imu_data_yaw;
    imu_data.linear_acceleration.x = robot_state->xyz_acc[0];
    imu_data.linear_acceleration.y = robot_state->xyz_acc[1];
    imu_data.linear_acceleration.z = robot_state->xyz_acc[2];
    imu_data.angular_velocity.x = robot_state->rpy_vel[0];
    imu_data.angular_velocity.y = robot_state->rpy_vel[1];
    imu_data.angular_velocity.z = robot_state->rpy_vel[2];
    imu_pub.publish(imu_data);

    /******leg_odom_data*******/
    geometry_msgs::TransformStamped leg_odom_trans;
    leg_odom_trans.header.stamp = current_time;
    leg_odom_trans.header.frame_id = "odom";
    leg_odom_trans.child_frame_id = "base_link";
    leg_odom_trans.transform.translation.x = robot_state->pos_world[0];
    leg_odom_trans.transform.translation.y = robot_state->pos_world[1];
    leg_odom_trans.transform.translation.z = 0.0;
    leg_odom_trans.transform.rotation = imu_data_yaw;
    // 采用 map -> velodyne -> base_link
    // odom_broadcaster.sendTransform(leg_odom_trans);

    // Position
    leg_odom_data.header.stamp = current_time;
    leg_odom_data.pose.pose.orientation = imu_data_yaw;
    leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
    leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
    leg_odom_data.pose.pose.position.z = 0.0;

    leg_pose_data.header.stamp = current_time;
    leg_pose_data.pose.pose.orientation = imu_data_qua;
    leg_pose_data.pose.pose.position.x = robot_state->pos_world[0];
    leg_pose_data.pose.pose.position.y = robot_state->pos_world[1];
    leg_pose_data.pose.pose.position.z = robot_state->pos_world[2];

    // Velocity
    double theta = robot_state->rpy[2] / 180 * PI;
    filter_vel_x.in(robot_state->vel_world[0]);
    filter_vel_y.in(robot_state->vel_world[1]);
    filter_vel_theta.in(robot_state->rpy_vel[2]);
    leg_odom_data.twist.twist.linear.x =
        filter_vel_x.out() * cos(theta) + filter_vel_y.out() * sin(theta);
    leg_odom_data.twist.twist.linear.y =
        -filter_vel_x.out() * sin(theta) + filter_vel_y.out() * cos(theta);
    leg_odom_data.twist.twist.angular.z = filter_vel_theta.out();

    leg_odom_pub.publish(leg_odom_data);
    leg_pose_pub.publish(leg_pose_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  close(sock_fd);
  return 0;
}

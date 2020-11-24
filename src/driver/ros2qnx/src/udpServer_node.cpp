#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ctime>
#include <stdlib.h>

#include "input.h"

using namespace std;

boost::shared_ptr<InputSocket> input_;
int packet_size_ = 0;

double last_time_cmd_vel = 0;

// tf::TransformListener robot_tf_listener;
// tf::StampedTransform robot_transform;

float robot_currnet_x = 0.0;
float robot_currnet_y = 0.0;
float robot_currnet_yaw = 0.0;
float robot_lidar_currnet_x = 0.0;
float robot_lidar_currnet_y = 0.0;
float robot_lidar_currnet_yaw = 0.0;
float robot_apriltag_currnet_x = 0.0;
float robot_apriltag_currnet_y = 0.0;
float robot_apriltag_currnet_yaw = 0.0;
float robot_apriltag_currnet_x_last = 0.0;
float robot_apriltag_currnet_y_last = 0.0;
float robot_apriltag_currnet_yaw_last = 0.0;

struct ball_position {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
} ball_position_;

bool in_charge_region = false;
// bool first_apriltag_pose = true;
int apriltag_call_count = 0;
int apriltag_call_count_last = 0;
int circle_count = 0;

float vel_x = 0;
float vel_y = 0;
float vth_z = 0;
float vel_x_old = 0;
float vel_y_old = 0;
float vth_z_old = 0;
float vel_x_now = 0;
float vel_y_now = 0;
float vth_z_now = 0;

float acc_x = 1.5;
float acc_y = 1.0;
float ath_z = 1.0;

//  flag
bool need_charge_flag = false;
bool charging_flag = false;

// 设定为 4 字节对齐
#pragma pack(4)
struct DataSend {
  int code;
  int cmd_data;
  int cons_code;
};

void ball_pose_Callback(geometry_msgs::PoseStampedConstPtr msg) {
  ball_position_.x = msg->pose.position.x;
  ball_position_.y = msg->pose.position.y;
  ball_position_.z = msg->pose.position.z;
  // std::cout << "ball_pose_Callback::" << ball_position_.x << "    " <<
  // ball_position_.y << "   " << ball_position_.z << "\n";
}

void ndt_pose_Callback(geometry_msgs::PoseStampedConstPtr msg) {
  robot_lidar_currnet_x = msg->pose.position.x;
  robot_lidar_currnet_y = msg->pose.position.y;
  geometry_msgs::Quaternion orientation = msg->pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z,
                                   orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  robot_lidar_currnet_yaw = yaw;
}

void apriltag_pose_Callback(geometry_msgs::PoseStampedConstPtr msg) {
  robot_apriltag_currnet_x = msg->pose.position.x;
  robot_apriltag_currnet_y = msg->pose.position.y;
  geometry_msgs::Quaternion orientation = msg->pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z,
                                   orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  robot_apriltag_currnet_yaw = yaw;
  // in_charge_region = true;
  apriltag_call_count++;
  if (apriltag_call_count > 100) apriltag_call_count = 0;
}

/**
 * @brief "cmd_vel"topic的回调函数，并通过udp发送数据
 * @param msg
 */

void arrive_charge_Callback(std_msgs::String msg) {
  if (msg.data == "yes") {
    in_charge_region = true;
  } else if (msg.data == "no") {
    in_charge_region = false;
  }
}

void vel_callback(geometry_msgs::TwistConstPtr msg) {
  last_time_cmd_vel = ros::Time::now().toSec();
  vel_x_old = vel_x;
  vel_y_old = vel_y;
  vth_z_old = vth_z;
  // double delta_x = msg->linear.x - vel_x_old;
  // if (abs(vel_x_old) < 0.2 && abs(delta_x) > 0.05) {
    vel_x = msg->linear.x;
  // }
  // if (abs(vel_x_old) >= 0.2 && abs(delta_x) > 0.1) {
  //   vel_x = msg->linear.x;
  // }
  
  vel_y = 0.3 * msg->linear.y;
  
  // double delta_z = msg->angular.z - vth_z_old;
  // if (abs(vth_z_old) < 0.2 && abs(delta_z) > 0.05) {
    vth_z = msg->angular.z;
  // }
  // if (abs(vth_z_old) >= 0.2 && abs(delta_z) > 0.1) {
  //   vth_z = msg->angular.z;
  // }
}

void leg_odom_callback(nav_msgs::OdometryConstPtr msg) {
  vel_x_now = msg->twist.twist.linear.x;
  vel_y_now = msg->twist.twist.linear.y;
  vth_z_now = msg->twist.twist.angular.z;

  // frequency of cmd_vel == 5hz
  // acc_x = (vel_x - vel_x_now) * 5;
  // acc_y = (vel_y - vel_y_now) * 5;
  // ath_z = (vth_z - vth_z_now) * 5;
}

void acc_callback(geometry_msgs::AccelConstPtr msg) {
  acc_x = msg->linear.x;
  acc_y = msg->linear.y;
  ath_z = msg->angular.z;
}

void recvUDPThread() {
  ros::Rate rate(1000);

  /* 开始接收数据 */
  while (ros::ok()) {
    uint8_t tmp_packet[packet_size_];
    while (true) {
      int rc = input_->getPacket(tmp_packet, packet_size_);

      if (rc == 0) break;                         // got a full packet?
      if (rc < 0) ROS_ERROR("getPacket Error!");  // end of file reached?
    }

    std::cout << tmp_packet << endl;
    // input_->sendPacket(tmp_packet, packet_size_);
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle n11;
  ros::NodeHandle private_nh("~");

  ros::Subscriber vel_sub = n11.subscribe("cmd_vel", 10, vel_callback);
  ros::Subscriber leg_odom_pub = n11.subscribe("leg_odom", 10, leg_odom_callback);
  ros::Subscriber acc_sub = n11.subscribe("acceleration", 10, acc_callback);

  ros::Subscriber arrive_charge_pose_sub =
      n11.subscribe("/if_arrive_charge", 10, arrive_charge_Callback);
  ros::Subscriber apriltag_pose_sub =
      n11.subscribe("/pose_in_apriltag", 10, apriltag_pose_Callback);
  ros::Subscriber ball_pose_sub =
      n11.subscribe("/ball_position", 10, ball_pose_Callback);

  private_nh.param<int>("packet_size", packet_size_, 10);

  float initPose_x, initPose_y, initPose_th, goalPose_x, goalPose_y,
      goalPose_th;
  private_nh.param<float>("initPose_x", initPose_x, 0.0);
  private_nh.param<float>("initPose_y", initPose_y, 0.0);
  private_nh.param<float>("initPose_th", initPose_th, 0.0);
  private_nh.param<float>("goalPose_x", goalPose_x, 0.0);
  private_nh.param<float>("goalPose_y", goalPose_y, 0.0);
  private_nh.param<float>("goalPose_th", goalPose_th, 0.0);

  DataSend data;

  // read data from live socket
  input_.reset(new InputSocket(private_nh));

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    // char buff[512] = {0};
    // uint8_t msg_len = 0;
    // // 帧头
    // buff[msg_len++] = 14;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;

    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;

    // buff[msg_len++] = 1;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;
    // buff[msg_len++] = 0;

    // if (in_charge_region) {
    //   std::cout << "in_charge_region : " << in_charge_region << endl;
    //   std::cout << "april------" << robot_apriltag_currnet_x << "     "
    //             << robot_apriltag_currnet_y << "   "
    //             << robot_apriltag_currnet_yaw << endl;
    // }
    // if (in_charge_region)  // in apriltag rigion
    // {
    //   // if the value didn't update
    //   if (robot_apriltag_currnet_x == robot_apriltag_currnet_x_last &&
    //       robot_apriltag_currnet_y == robot_apriltag_currnet_y_last &&
    //       robot_apriltag_currnet_yaw == robot_apriltag_currnet_yaw_last) {
    //     robot_apriltag_currnet_x = robot_apriltag_currnet_y =
    //         robot_apriltag_currnet_yaw = 0.0;
    //   } else {
    //     robot_apriltag_currnet_x_last = robot_apriltag_currnet_x;
    //     robot_apriltag_currnet_y_last = robot_apriltag_currnet_y;
    //     robot_apriltag_currnet_yaw_last = robot_apriltag_currnet_yaw;
    //   }
    //   robot_currnet_x = robot_apriltag_currnet_x;
    //   robot_currnet_y = robot_apriltag_currnet_y;
    //   robot_currnet_yaw = robot_apriltag_currnet_yaw;
    // } else {
    //   robot_currnet_x = robot_lidar_currnet_x;
    //   robot_currnet_y = robot_lidar_currnet_y;
    //   robot_currnet_yaw = robot_lidar_currnet_yaw;
    // }
    // 以下开始为消息
    // msg_len += sprintf(buff + msg_len, "%.3f", vel_x);               // 0
    // msg_len += sprintf(buff + msg_len, ",%.3f", vel_y);              // 1
    // msg_len += sprintf(buff + msg_len, ",%.3f", vth_z);              // 2
    // msg_len += sprintf(buff + msg_len, ",%d", 0);                    // 3
    // msg_len += sprintf(buff + msg_len, ",%d", 0);                    // 4
    // msg_len += sprintf(buff + msg_len, ",%d", 0);                    // 5
    // msg_len += sprintf(buff + msg_len, ",%.3f", 0.0);                // 6
    // msg_len += sprintf(buff + msg_len, ",%.3f", robot_currnet_x);    // 7
    // msg_len += sprintf(buff + msg_len, ",%.3f", robot_currnet_y);    // 8
    // msg_len += sprintf(buff + msg_len, ",%.3f", robot_currnet_yaw);  // 9
    // msg_len += sprintf(buff + msg_len, ",%d", in_charge_region);     // 10
    // msg_len += sprintf(buff + msg_len, ",%.3f", ball_position_.x);   // 11
    // msg_len += sprintf(buff + msg_len, ",%.3f", ball_position_.y);   // 12
    // msg_len += sprintf(buff + msg_len, ",%.3f", ball_position_.z);   // 13

    // buff[4] = msg_len - 12;  // 消息的长度填充到第二个字节
    // input_->sendPacket((uint8_t*)buff, msg_len);

    // 速度V
    if (true) {
      if ((ros::Time::now().toSec() - last_time_cmd_vel) > 0.25) {
        vel_x = 0;
        vel_y = 0;
        vth_z = 0;
        // acc_x = 1.5;
        // acc_y = 1.0;
        // ath_z = 1.0;
        // ROS_WARN("STOP! cmd_vel commands over 0.3s.");
      }
      data.code = 291;
      data.cons_code = 0;
      data.cmd_data = vel_x * 1.20 * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
      data.code = 292;
      data.cons_code = 0;
      data.cmd_data = vel_y * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
      data.code = 290;
      data.cons_code = 0;
      data.cmd_data = vth_z * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
    }

    // 加速度
    if (false) {
      data.code = 0x0126;
      data.cons_code = 0;
      data.cmd_data = acc_x * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
      data.code = 0x0127;
      data.cons_code = 0;
      data.cmd_data = acc_y * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
      data.code = 0x0125;
      data.cons_code = 0;
      data.cmd_data = ath_z * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
      // ROS_INFO("acc_x: %f, acc_y: %f, ath_z: %f", acc_x, acc_y, ath_z);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

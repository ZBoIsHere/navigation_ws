#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <ctime>

using namespace std;

double last_time_cmd_vel = 0;

float vel_x = 0;
float vel_y = 0;
float vth_z = 0;

// 设定为 4 字节对齐
#pragma pack(4)
struct DataSend {
  int code;
  int cmd_data;
  int cons_code;
};

void vel_callback(geometry_msgs::TwistConstPtr msg) {
  last_time_cmd_vel = ros::Time::now().toSec();
  vel_x = msg->linear.x;
  vel_y = msg->linear.y;
  vth_z = msg->angular.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sender");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, vel_callback);

  int remote_port;
  std::string remote_ip;
  private_nh.param<int>("remote_port", remote_port, 43893);
  private_nh.param("remote_ip", remote_ip, std::string(""));

  int sockfd;
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket creation failed");
    return 0;
  }
  sockaddr_in remote_addr;
  memset(&remote_addr, 0, sizeof(remote_addr));
  remote_addr.sin_family = AF_INET;
  remote_addr.sin_port = htons(remote_port);
  remote_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());

  DataSend data;
  ssize_t nbytes;
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    // 速度V
    if (true) {
      if ((ros::Time::now().toSec() - last_time_cmd_vel) > 0.25) {
        vel_x = 0;
        vel_y = 0;
        vth_z = 0;
      }
      data.code = 290;
      data.cons_code = 0;
      data.cmd_data = vth_z * 1000;
      nbytes = sendto(sockfd, (uint8_t *)&data, sizeof(data), 0,
                      (struct sockaddr *)&remote_addr, sizeof(remote_addr));
      if (nbytes < 0) {
        perror("sendfail");
        ROS_INFO("sendfail");
        return 0;
      }
      data.code = 291;
      data.cons_code = 0;
      data.cmd_data = vel_x * 1000;
      nbytes = sendto(sockfd, (uint8_t *)&data, sizeof(data), 0,
                      (struct sockaddr *)&remote_addr, sizeof(remote_addr));
      if (nbytes < 0) {
        perror("sendfail");
        ROS_INFO("sendfail");
        return 0;
      }

      data.code = 292;
      data.cons_code = 0;
      data.cmd_data = vel_y * 1000;
      nbytes = sendto(sockfd, (uint8_t *)&data, sizeof(data), 0,
                      (struct sockaddr *)&remote_addr, sizeof(remote_addr));
      if (nbytes < 0) {
        perror("sendfail");
        ROS_INFO("sendfail");
        return 0;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  close(sockfd);
  return 0;
}

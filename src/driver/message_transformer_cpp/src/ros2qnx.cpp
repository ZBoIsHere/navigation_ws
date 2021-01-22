#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <ctime>

#include "input.h"

using namespace std;

boost::shared_ptr<InputSocket> input_;

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, vel_callback);
  double vel_x_factor;
  nh.param<double>("vel_x_factor", vel_x_factor, 1.0);

  input_.reset(new InputSocket(private_nh));

  DataSend data;
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    // 速度V
    if (true) {
      if ((ros::Time::now().toSec() - last_time_cmd_vel) > 0.25) {
        vel_x = 0;
        vel_y = 0;
        vth_z = 0;
        // ROS_WARN("STOP! cmd_vel commands over 0.3s.");
      }
      data.code = 290;
      data.cons_code = 0;
      data.cmd_data = vth_z * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));

      data.code = 291;
      data.cons_code = 0;
      data.cmd_data = vel_x * 1000 * vel_x_factor;
      input_->sendPacket((uint8_t*)&data, sizeof(data));

      data.code = 292;
      data.cons_code = 0;
      data.cmd_data = vel_y * 1000;
      input_->sendPacket((uint8_t*)&data, sizeof(data));
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <string>

using namespace std;

class SubscribeAndPublish {
 public:
  SubscribeAndPublish(ros::NodeHandle nh) : n_(nh) {
    pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
    sub_ =
        n_.subscribe("/joint_angle", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const std_msgs::Float64MultiArray& input) {
    sensor_msgs::JointState joint_state;
    //.... do something with the input and generate the output...
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);

    joint_state.name[0] = "LF_Joint";
    joint_state.position[0] = -input.data[0];

    joint_state.name[1] = "LF_Joint_1";
    joint_state.position[1] = -input.data[1];

    joint_state.name[2] = "LF_Joint_2";
    joint_state.position[2] = -input.data[2];

    joint_state.name[3] = "LB_Joint";
    joint_state.position[3] = -input.data[3];

    joint_state.name[4] = "LB_Joint_1";
    joint_state.position[4] = -input.data[4];

    joint_state.name[5] = "LB_Joint_2";
    joint_state.position[5] = -input.data[5];

    joint_state.name[6] = "RF_Joint";
    joint_state.position[6] = -input.data[6];

    joint_state.name[7] = "RF_Joint_1";
    joint_state.position[7] = -input.data[7];

    joint_state.name[8] = "RF_Joint_2";
    joint_state.position[8] = -input.data[8];

    joint_state.name[9] = "RB_Joint";
    joint_state.position[9] = -input.data[9];

    joint_state.name[10] = "RB_Joint_1";
    joint_state.position[10] = -input.data[10];

    joint_state.name[11] = "RB_Joint_2";
    joint_state.position[11] = -input.data[11];

    pub_.publish(joint_state);
  }

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv) {
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
  ros::NodeHandle nh;
  SubscribeAndPublish JointPublish(nh);
  ros::spin();
  return 0;
}

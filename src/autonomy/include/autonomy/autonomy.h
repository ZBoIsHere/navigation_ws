#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <actionlib/client/simple_action_client.h>
#include <arpa/inet.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <netinet/in.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "autonomy/json.hpp"

using json = nlohmann::json;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class QPushButton;
class QLabel;

namespace autonomy {
class RobotCommander {
 public:
  RobotCommander(int remote_port, std::string remote_ip) {
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("socket creation failed");
    }
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(remote_port);
    remote_addr_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  }

  void changeGait() {
    Command data;
    data.command_code = 26;
    data.command_value = 0;
    data.command_type = 0;
    ssize_t nbytes;
    nbytes = sendto(sockfd_, (uint8_t*)&data, sizeof(data), 0,
                    (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (nbytes < 0) {
      perror("sendfail");
      ROS_INFO("sendfail");
    }
  }

  void resetGait() {
    Command data;
    data.command_code = 26;
    data.command_value = 0;
    data.command_type = 0;
    ssize_t nbytes;
    nbytes = sendto(sockfd_, (uint8_t*)&data, sizeof(data), 0,
                    (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (nbytes < 0) {
      perror("sendfail");
      ROS_INFO("sendfail");
    }
  }

  ~RobotCommander() { close(sockfd_); }

  int sockfd_;
  sockaddr_in remote_addr_;

  struct Command {
    int command_code;
    int command_value;
    int command_type;
  };
};

class Autonomy : public rviz::Panel {
  Q_OBJECT
 public:
  explicit Autonomy(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:

 protected Q_SLOTS:
  void addOneWaypoint();
  void saveAllWaypoints();
  void runAutonomy();
  void stopAutonomy();
  void showWaypoints();
  void visualization(const json&);
  void showString(const std::string&);
  void tick();

 protected:
  QPushButton* button_add;
  QPushButton* button_save;
  QPushButton* button_run;
  QPushButton* button_stop;
  QLabel* mid_label;

  int counter_teach_;
  int counter_repeat_;
  json added_j_;
  json saved_j_;

  ros::NodeHandle nh_;
  ros::Publisher waypoints_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  RobotCommander rc_;

  MoveBaseClient ac_;
  move_base_msgs::MoveBaseGoal goal_;

  enum FSM_EXEC_STATE { NO_WAY, NORMAL, TEACH, REPEAT };
  FSM_EXEC_STATE exec_state_;
};
}  // end namespace autonomy

#endif  // AUTONOMY_H
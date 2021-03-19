#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>
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

  MoveBaseClient ac_;
  move_base_msgs::MoveBaseGoal goal_;

  enum FSM_EXEC_STATE { NO_WAY, NORMAL, TEACH, REPEAT };
  FSM_EXEC_STATE exec_state_;
};

class RobotCommander {
  // TODO UDP
}

}  // end namespace autonomy

#endif  // AUTONOMY_H
#ifndef AUTONOMY_H
#define AUTONOMY_H

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
  void showAllWaypoints();

 protected:
  QPushButton* button_add_;
  QPushButton* button_save_;
  QPushButton* button_run_;
  QPushButton* button_stop_;
  QLabel* mid_gui_label_;

  json added_j_;
  int counter_;

  json fixed_j_;

  ros::NodeHandle nh_;
  ros::Publisher waypoints_publisher_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  enum FSM_EXEC_STATE { INIT, RECORD, REPEAT, ERROR };
  FSM_EXEC_STATE exec_state_;
};

}  // end namespace autonomy

#endif  // AUTONOMY_H
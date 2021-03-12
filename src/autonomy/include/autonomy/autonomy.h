#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
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

 protected:
  QPushButton* btn_add_;
  QPushButton* btn_save_;
  QPushButton* btn_run_;
  QPushButton* btn_stop_;
  QLabel* show_added_waypoint_;
  QLabel* show_total_waypoint_;

  json j_;
  int from_id_;
  int next_id_;
  int count_waypoints_;
};

}  // end namespace autonomy

#endif  // AUTONOMY_H
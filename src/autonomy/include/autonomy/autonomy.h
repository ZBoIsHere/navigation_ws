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
  QLabel* mid_gui_label_;

  json added_j_;
  int counter_;

  json fixed_j_;
};

}  // end namespace autonomy

#endif  // AUTONOMY_H
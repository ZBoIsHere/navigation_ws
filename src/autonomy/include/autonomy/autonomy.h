#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>

#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

class QPushButton;

namespace autonomy
{
class Autonomy : public rviz::Panel
{
Q_OBJECT
public:
  explicit Autonomy(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

  void moveNext();

  void moveAuto();

  void moveFullAuto();

  void moveStop();

protected:
  QPushButton* btn_add_;
  QPushButton* btn_save_;
  QPushButton* btn_run_;
  QPushButton* btn_stop_;
  QLineEdit* added_waypoint_;
  QLineEdit* total_waypoint_;
};

}  // end namespace autonomy

#endif  // AUTONOMY_H
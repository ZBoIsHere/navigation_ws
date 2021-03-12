#include "autonomy/autonomy.h"

#include <QLabel>
#include <iostream>
#include <string>

namespace autonomy {
Autonomy::Autonomy(QWidget* parent)
    : rviz::Panel(parent), from_id_(0), next_id_(0), count_waypoints_(0) {
  btn_add_ = new QPushButton(this);
  btn_add_->setText("Add");
  btn_save_ = new QPushButton(this);
  btn_save_->setText("Save");
  btn_run_ = new QPushButton(this);
  btn_run_->setText("Run");
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");

  connect(btn_add_, SIGNAL(clicked()), this, SLOT(addOneWaypoint()));
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(saveAllWaypoints()));
  connect(btn_run_, SIGNAL(clicked()), this, SLOT(runAutonomy()));
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(stopAutonomy()));

  show_added_waypoint_ = new QLabel("0");
  show_total_waypoint_ = new QLabel("0");

  auto* top = new QHBoxLayout;
  top->addWidget(btn_add_);
  top->addWidget(btn_save_);

  auto* mid = new QHBoxLayout;
  mid->addWidget(new QLabel("Added:"));
  mid->addWidget(show_added_waypoint_);
  mid->addWidget(new QLabel("Total:"));
  mid->addWidget(show_total_waypoint_);

  auto* bot = new QHBoxLayout;
  bot->addWidget(btn_run_);
  bot->addWidget(btn_stop_);

  auto* layout = new QVBoxLayout;
  layout->addLayout(top);
  layout->addLayout(mid);
  layout->addLayout(bot);
  setLayout(layout);

  btn_add_->setEnabled(true);
  btn_save_->setEnabled(true);
  btn_run_->setEnabled(true);
  btn_stop_->setEnabled(true);
}

void Autonomy::addOneWaypoint() {
  next_id_ = from_id_ + 1;
  j_[std::to_string(from_id_)]["x"] = 1.0;
  j_[std::to_string(from_id_)]["y"] = 2.0;
  j_[std::to_string(from_id_)]["theta"] = 3.0;
  j_[std::to_string(from_id_)]["next"] = next_id_;
  ++from_id_;
  ++count_waypoints_;

  std::string added_id = std::to_string(from_id_);
  QString added_str = QString::fromUtf8(added_id.c_str());
  show_added_waypoint_->setText(added_str);

  std::string count_id = std::to_string(count_waypoints_);
  QString count_str = QString::fromUtf8(count_id.c_str());
  show_total_waypoint_->setText(count_str);
}

void Autonomy::saveAllWaypoints() {
  std::cout << std::setw(4) << j_ << std::endl;
}

void Autonomy::runAutonomy() {}

void Autonomy::stopAutonomy() {}

void Autonomy::save(rviz::Config config) const { rviz::Panel::save(config); }

void Autonomy::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // end namespace autonomy

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomy::Autonomy, rviz::Panel)
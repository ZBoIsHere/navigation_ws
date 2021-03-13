#include "autonomy/autonomy.h"

#include <ros/package.h>

#include <fstream>
#include <iostream>
#include <string>

namespace autonomy {
Autonomy::Autonomy(QWidget* parent) : rviz::Panel(parent), counter_(0) {
  added_j_[std::to_string(0)]["total"] = 0;

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

  mid_gui_label_ = new QLabel("");

  auto* top = new QHBoxLayout;
  top->addWidget(btn_add_);
  top->addWidget(btn_save_);
  auto* mid = new QHBoxLayout;
  mid->addWidget(mid_gui_label_);
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
  ++counter_;
  added_j_[std::to_string(counter_)]["x"] = 1.0;
  added_j_[std::to_string(counter_)]["y"] = 2.0;
  added_j_[std::to_string(counter_)]["theta"] = 3.0;
  added_j_[std::to_string(counter_)]["next"] = counter_ + 1;

  added_j_[std::to_string(0)]["total"] = counter_;

  std::string added_id = "ADD No. " + std::to_string(counter_) + " Waypoint.";
  QString added_info = QString::fromUtf8(added_id.c_str());
  mid_gui_label_->clear();
  mid_gui_label_->setText(added_info);
}

void Autonomy::saveAllWaypoints() {
  std::string path_package = ros::package::getPath("autonomy");
  // LOOP CLOSURE
  added_j_[std::to_string(counter_)]["next"] = 1;
  std::string path_file = path_package + "/data/waypoints.json";
  std::ofstream o(path_file);
  o << std::setw(4) << added_j_ << std::endl;

  std::string count_id = "SAVED " + std::to_string(counter_) + " Waypoints.";
  QString count_info = QString::fromUtf8(count_id.c_str());
  mid_gui_label_->clear();
  mid_gui_label_->setText(count_info);
}

void Autonomy::runAutonomy() {
  std::string path_package = ros::package::getPath("autonomy");
  std::string path_file = path_package + "/data/waypoints.json";
  std::ifstream i(path_file);
  i >> fixed_j_;

  int totol = fixed_j_["0"]["total"];
  std::string total_id = "LOADED " + std::to_string(totol) + " Waypoints.";
  QString total_str = QString::fromUtf8(total_id.c_str());
  mid_gui_label_->clear();
  mid_gui_label_->setText(total_str);

  added_j_.clear();
  counter_ = 0;

  std::cout << std::setw(4) << fixed_j_ << std::endl;
}

void Autonomy::stopAutonomy() {}

void Autonomy::save(rviz::Config config) const { rviz::Panel::save(config); }

void Autonomy::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // end namespace autonomy

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomy::Autonomy, rviz::Panel)
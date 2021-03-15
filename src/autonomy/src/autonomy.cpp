#include "autonomy/autonomy.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <QTimer>
#include <fstream>
#include <iostream>
#include <string>

namespace autonomy {
Autonomy::Autonomy(QWidget* parent)
    : rviz::Panel(parent),
      counter_(0),
      tfListener_(tfBuffer_),
      exec_state_(INIT) {
  added_j_[std::to_string(0)]["total"] = 0;

  button_add_ = new QPushButton(this);
  button_add_->setText("Add");
  button_save_ = new QPushButton(this);
  button_save_->setText("Save");
  button_run_ = new QPushButton(this);
  button_run_->setText("Run");
  button_stop_ = new QPushButton(this);
  button_stop_->setText("Stop");

  connect(button_add_, SIGNAL(clicked()), this, SLOT(addOneWaypoint()));
  connect(button_save_, SIGNAL(clicked()), this, SLOT(saveAllWaypoints()));
  connect(button_run_, SIGNAL(clicked()), this, SLOT(runAutonomy()));
  connect(button_stop_, SIGNAL(clicked()), this, SLOT(stopAutonomy()));

  mid_gui_label_ = new QLabel("");

  auto* top = new QHBoxLayout;
  top->addWidget(button_add_);
  top->addWidget(button_save_);
  auto* mid = new QHBoxLayout;
  mid->addWidget(mid_gui_label_);
  auto* bot = new QHBoxLayout;
  bot->addWidget(button_run_);
  bot->addWidget(button_stop_);

  auto* layout = new QVBoxLayout;
  layout->addLayout(top);
  layout->addLayout(mid);
  layout->addLayout(bot);
  setLayout(layout);

  button_add_->setEnabled(true);
  button_save_->setEnabled(true);
  button_run_->setEnabled(true);
  button_stop_->setEnabled(true);

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(showAllWaypoints()));
  output_timer->start(1000);

  waypoints_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1);

  std::string path_package = ros::package::getPath("autonomy");
  std::string path_file = path_package + "/data/waypoints.json";
  std::ifstream i(path_file);
  if (!i || (i.peek() == std::ifstream::traits_type::eof())) {
    std::string error_id = "FIND Zero Waypoint";
    QString error_str = QString::fromUtf8(error_id.c_str());
    mid_gui_label_->clear();
    mid_gui_label_->setText(error_str);
    exec_state_ = ERROR;
  } else {
    i >> fixed_j_;
    int totol = fixed_j_["0"]["total"];
    std::string total_id = "FIND " + std::to_string(totol) + " Waypoints.";
    QString total_str = QString::fromUtf8(total_id.c_str());
    mid_gui_label_->clear();
    mid_gui_label_->setText(total_str);
    exec_state_ = INIT;
  }
}

void Autonomy::addOneWaypoint() {
  exec_state_ = RECORD;
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
        tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
  tf2::Quaternion quat_tf;
  tf2::convert(transformStamped.transform.rotation, quat_tf);
  tf2::Matrix3x3 m(quat_tf);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ++counter_;
  added_j_[std::to_string(counter_)]["x"] =
      transformStamped.transform.translation.x;
  added_j_[std::to_string(counter_)]["y"] =
      transformStamped.transform.translation.y;
  added_j_[std::to_string(counter_)]["theta"] = yaw;
  added_j_[std::to_string(counter_)]["next"] = counter_ + 1;

  added_j_[std::to_string(0)]["total"] = counter_;

  std::string added_id = "ADD No. " + std::to_string(counter_) + " Waypoint.";
  QString added_info = QString::fromUtf8(added_id.c_str());
  mid_gui_label_->clear();
  mid_gui_label_->setText(added_info);
}

void Autonomy::saveAllWaypoints() {
  exec_state_ = RECORD;
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
  exec_state_ = REPEAT;
  std::string path_package = ros::package::getPath("autonomy");
  std::string path_file = path_package + "/data/waypoints.json";
  std::ifstream i(path_file);
  if (!i || (i.peek() == std::ifstream::traits_type::eof())) {
    std::string error_id = "ERROR: Zero Waypoint";
    QString error_str = QString::fromUtf8(error_id.c_str());
    mid_gui_label_->clear();
    mid_gui_label_->setText(error_str);
    exec_state_ = ERROR;
  } else {
    i >> fixed_j_;
    int totol = fixed_j_["0"]["total"];
    std::string total_id = "LOADED " + std::to_string(totol) + " Waypoints.";
    QString total_str = QString::fromUtf8(total_id.c_str());
    mid_gui_label_->clear();
    mid_gui_label_->setText(total_str);

    added_j_.clear();
    counter_ = 0;

    std::cout << std::setw(4) << fixed_j_ << std::endl;
    // TODO BT
  }
}

void Autonomy::stopAutonomy() { exec_state_ = REPEAT; }

void Autonomy::showAllWaypoints() {
  visualization_msgs::MarkerArray waypoints_list;
  switch (exec_state_) {
    case INIT: {
      int length = fixed_j_["0"]["total"];
      waypoints_list.markers.resize(length);

      for (int i = 0; i < length; ++i) {
        waypoints_list.markers[i].header.frame_id = "map";
        waypoints_list.markers[i].header.stamp = ros::Time::now();

        waypoints_list.markers[i].ns = "waypoints";
        waypoints_list.markers[i].id = i;

        waypoints_list.markers[i].type =
            visualization_msgs::Marker::TEXT_VIEW_FACING;
        waypoints_list.markers[i].action = visualization_msgs::Marker::ADD;
        waypoints_list.markers[i].pose.position.x = 1.0 * i;
        waypoints_list.markers[i].pose.position.y = 1;
        waypoints_list.markers[i].pose.position.z = 1;
        waypoints_list.markers[i].pose.orientation.x = 0.0;
        waypoints_list.markers[i].pose.orientation.y = 0.0;
        waypoints_list.markers[i].pose.orientation.z = 0.0;
        waypoints_list.markers[i].pose.orientation.w = 1.0;
        waypoints_list.markers[i].scale.x = 0.5;
        waypoints_list.markers[i].scale.y = 0.5;
        waypoints_list.markers[i].scale.z = 0.5;
        waypoints_list.markers[i].color.a = 1.0;
        waypoints_list.markers[i].color.r = 1.0;
        waypoints_list.markers[i].color.g = 0.0;
        waypoints_list.markers[i].color.b = 0.0;
        waypoints_list.markers[i].text = std::to_string(i);
      }
      break;
    }
    case RECORD: {
      int length = added_j_["0"]["total"];
      waypoints_list.markers.resize(length);

      for (int i = 0; i < length; ++i) {
        waypoints_list.markers[i].header.frame_id = "map";
        waypoints_list.markers[i].header.stamp = ros::Time::now();

        waypoints_list.markers[i].ns = "waypoints";
        waypoints_list.markers[i].id = i;

        waypoints_list.markers[i].type =
            visualization_msgs::Marker::TEXT_VIEW_FACING;
        waypoints_list.markers[i].action = visualization_msgs::Marker::ADD;
        waypoints_list.markers[i].pose.position.x = 1.0 * i;
        waypoints_list.markers[i].pose.position.y = 1;
        waypoints_list.markers[i].pose.position.z = 1;
        waypoints_list.markers[i].pose.orientation.x = 0.0;
        waypoints_list.markers[i].pose.orientation.y = 0.0;
        waypoints_list.markers[i].pose.orientation.z = 0.0;
        waypoints_list.markers[i].pose.orientation.w = 1.0;
        waypoints_list.markers[i].scale.x = 0.5;
        waypoints_list.markers[i].scale.y = 0.5;
        waypoints_list.markers[i].scale.z = 0.5;
        waypoints_list.markers[i].color.a = 1.0;
        waypoints_list.markers[i].color.r = 1.0;
        waypoints_list.markers[i].color.g = 0.0;
        waypoints_list.markers[i].color.b = 0.0;
        waypoints_list.markers[i].text = std::to_string(i);
      }
      break;
    }
    case REPEAT: {
      int length = fixed_j_["0"]["total"];
      waypoints_list.markers.resize(length);

      for (int i = 0; i < length; ++i) {
        waypoints_list.markers[i].header.frame_id = "map";
        waypoints_list.markers[i].header.stamp = ros::Time::now();

        waypoints_list.markers[i].ns = "waypoints";
        waypoints_list.markers[i].id = i;

        waypoints_list.markers[i].type =
            visualization_msgs::Marker::TEXT_VIEW_FACING;
        waypoints_list.markers[i].action = visualization_msgs::Marker::ADD;
        waypoints_list.markers[i].pose.position.x = 1.0 * i;
        waypoints_list.markers[i].pose.position.y = 1;
        waypoints_list.markers[i].pose.position.z = 1;
        waypoints_list.markers[i].pose.orientation.x = 0.0;
        waypoints_list.markers[i].pose.orientation.y = 0.0;
        waypoints_list.markers[i].pose.orientation.z = 0.0;
        waypoints_list.markers[i].pose.orientation.w = 1.0;
        waypoints_list.markers[i].scale.x = 0.5;
        waypoints_list.markers[i].scale.y = 0.5;
        waypoints_list.markers[i].scale.z = 0.5;
        waypoints_list.markers[i].color.a = 1.0;
        waypoints_list.markers[i].color.r = 1.0;
        waypoints_list.markers[i].color.g = 0.0;
        waypoints_list.markers[i].color.b = 0.0;
        waypoints_list.markers[i].text = std::to_string(i);
      }
      break;
    }
  }

  if (waypoints_publisher_.getNumSubscribers() < 1) {
    return;
  } else {
    waypoints_publisher_.publish(waypoints_list);
  }
}

void Autonomy::save(rviz::Config config) const { rviz::Panel::save(config); }

void Autonomy::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // end namespace autonomy

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomy::Autonomy, rviz::Panel)
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
      counter_teach_(0),
      counter_repeat_(1),
      tf_listener_(tf_buffer_),
      exec_state_(NO_WAY),
      ac_("move_base", true) {
  added_j_[std::to_string(0)]["total"] = 0;

  button_add = new QPushButton(this);
  button_add->setText("Add");
  button_save = new QPushButton(this);
  button_save->setText("Save");
  button_run = new QPushButton(this);
  button_run->setText("Run");
  button_stop = new QPushButton(this);
  button_stop->setText("Stop");
  connect(button_add, SIGNAL(clicked()), this, SLOT(addOneWaypoint()));
  connect(button_save, SIGNAL(clicked()), this, SLOT(saveAllWaypoints()));
  connect(button_run, SIGNAL(clicked()), this, SLOT(runAutonomy()));
  connect(button_stop, SIGNAL(clicked()), this, SLOT(stopAutonomy()));

  mid_label = new QLabel("");

  auto* top = new QHBoxLayout;
  top->addWidget(button_add);
  top->addWidget(button_save);
  auto* mid = new QHBoxLayout;
  mid->addWidget(mid_label);
  auto* bot = new QHBoxLayout;
  bot->addWidget(button_run);
  bot->addWidget(button_stop);

  auto* layout = new QVBoxLayout;
  layout->addLayout(top);
  layout->addLayout(mid);
  layout->addLayout(bot);
  setLayout(layout);

  button_add->setEnabled(true);
  button_save->setEnabled(true);
  button_run->setEnabled(true);
  button_stop->setEnabled(true);

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(showWaypoints()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  waypoints_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
  std::string path_package = ros::package::getPath("autonomy");
  std::string path_file = path_package + "/data/waypoints.json";
  std::ifstream i(path_file);
  if (!i || (i.peek() == std::ifstream::traits_type::eof())) {
    std::string s = "FIND 0 Waypoint";
    showString(s);
    exec_state_ = NO_WAY;
  } else {
    i >> saved_j_;
    int totol = saved_j_["0"]["total"];
    std::string s = "FIND " + std::to_string(totol) + " Waypoints.";
    showString(s);
    exec_state_ = NORMAL;
  }
}

void Autonomy::addOneWaypoint() {
  exec_state_ = TEACH;
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
        tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
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

  added_j_[std::to_string(0)]["total"] = ++counter_teach_;
  added_j_[std::to_string(counter_teach_)]["x"] =
      transformStamped.transform.translation.x;
  added_j_[std::to_string(counter_teach_)]["y"] =
      transformStamped.transform.translation.y;
  added_j_[std::to_string(counter_teach_)]["z"] =
      transformStamped.transform.translation.z;
  added_j_[std::to_string(counter_teach_)]["theta"] = yaw;
  added_j_[std::to_string(counter_teach_)]["next"] = counter_teach_ + 1;
  std::string s = "ADD No. " + std::to_string(counter_teach_) + " Waypoint.";
  showString(s);
}

void Autonomy::saveAllWaypoints() {
  std::string path_package = ros::package::getPath("autonomy");
  // LOOP CLOSURE
  added_j_[std::to_string(counter_teach_)]["next"] = 1;
  std::string path_file = path_package + "/data/waypoints.json";
  std::ofstream o(path_file);
  o << std::setw(4) << added_j_ << std::endl;
  std::string s = "SAVED " + std::to_string(counter_teach_) + " Waypoints.";
  showString(s);
}

void Autonomy::runAutonomy() {
  std::string path_package = ros::package::getPath("autonomy");
  std::string path_file = path_package + "/data/waypoints.json";
  std::ifstream i(path_file);
  if (!i || (i.peek() == std::ifstream::traits_type::eof())) {
    std::string s = "ERROR: 0 Waypoint";
    showString(s);
    exec_state_ = NO_WAY;
  } else {
    i >> saved_j_;
    std::cout << std::setw(4) << saved_j_ << std::endl;
    int totol = saved_j_["0"]["total"];
    std::string s = "LOADED " + std::to_string(totol) + " Waypoints.";
    showString(s);

    added_j_.clear();
    counter_teach_ = 0;

    if (!ac_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server!");
      if (exec_state_ == TEACH) exec_state_ = NORMAL;
    } else {
      exec_state_ = REPEAT;
    }
    // TODO BT
  }
}

void Autonomy::stopAutonomy() {
  exec_state_ = (exec_state_ == NO_WAY) ? NO_WAY : NORMAL;
  std::string s = "STOP!";
  showString(s);
}

void Autonomy::showWaypoints() {
  switch (exec_state_) {
    case TEACH:
      visualization(added_j_);
      break;
    case NORMAL:
    case REPEAT:
      visualization(saved_j_);
      break;
  }
}

void Autonomy::visualization(const json& j) {
  visualization_msgs::MarkerArray waypoints_list;
  int length = j["0"]["total"];
  waypoints_list.markers.resize(length);

  for (int i = 0; i < length; ++i) {
    waypoints_list.markers[i].header.frame_id = "map";
    waypoints_list.markers[i].header.stamp = ros::Time::now();

    waypoints_list.markers[i].ns = "waypoints";
    waypoints_list.markers[i].id = i + 1;

    waypoints_list.markers[i].type =
        visualization_msgs::Marker::TEXT_VIEW_FACING;
    waypoints_list.markers[i].action = visualization_msgs::Marker::ADD;
    waypoints_list.markers[i].pose.position.x = j[std::to_string(i + 1)]["x"];
    waypoints_list.markers[i].pose.position.y = j[std::to_string(i + 1)]["y"];
    waypoints_list.markers[i].pose.position.z = j[std::to_string(i + 1)]["z"];
    waypoints_list.markers[i].pose.orientation.x = 0.0;
    waypoints_list.markers[i].pose.orientation.y = 0.0;
    waypoints_list.markers[i].pose.orientation.z = 0.0;
    waypoints_list.markers[i].pose.orientation.w = 1.0;
    waypoints_list.markers[i].scale.z = 0.5;
    waypoints_list.markers[i].color.a = 1.0;
    waypoints_list.markers[i].color.r = 0.0;
    waypoints_list.markers[i].color.g = 0.0;
    waypoints_list.markers[i].color.b = 1.0;
    waypoints_list.markers[i].text = std::to_string(i + 1);
  }
  if (waypoints_publisher_.getNumSubscribers() > 0) {
    waypoints_publisher_.publish(waypoints_list);
  }
}

void Autonomy::showString(const std::string& s) {
  QString q_s = QString::fromUtf8(s.c_str());
  mid_label->clear();
  mid_label->setText(q_s);
}

void Autonomy::tick() {
  if (exec_state_ == REPEAT) {
    ROS_INFO("REPEAT");
  }
}

void Autonomy::save(rviz::Config config) const { rviz::Panel::save(config); }

void Autonomy::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // end namespace autonomy

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomy::Autonomy, rviz::Panel)
#include <cstdio>

#include <QLabel>

#include "autonomy/autonomy.h"

namespace autonomy
{
Autonomy::Autonomy(QWidget* parent) : rviz::Panel(parent)
{
  // Create a push button
  btn_add_ = new QPushButton(this);
  btn_add_->setText("Add");
  connect(btn_add_, SIGNAL(clicked()), this, SLOT(moveNext()));

  // Create a push button
  btn_save_ = new QPushButton(this);
  btn_save_->setText("Save");
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(moveAuto()));

  // Create a push button
  btn_run_ = new QPushButton(this);
  btn_run_->setText("Run");
  connect(btn_run_, SIGNAL(clicked()), this, SLOT(moveFullAuto()));

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Horizontal Layout
  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_add_);
  hlayout1->addWidget(btn_save_);

  auto* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(btn_run_);
  hlayout2->addWidget(btn_stop_);

  auto* left_layout = new QHBoxLayout;
  left_layout->addWidget( new QLabel( "Added:" ));
  added_waypoint_ = new QLineEdit;
  left_layout->addWidget( added_waypoint_ );

  auto* right_layout = new QHBoxLayout;
  right_layout->addWidget( new QLabel( "Total:" ));
  total_waypoint_ = new QLineEdit;
  right_layout->addWidget( total_waypoint_ );

  auto* mid = new QHBoxLayout;
  mid->addLayout(left_layout);
  mid->addLayout(right_layout);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addLayout(mid);
  layout->addLayout(hlayout2);
  setLayout(layout);

  btn_add_->setEnabled(true);
  btn_save_->setEnabled(true);
  btn_run_->setEnabled(true);
}

void Autonomy::moveNext()
{
  // remote_reciever_.publishNext();
}

void Autonomy::moveAuto()
{
  // remote_reciever_.publishContinue();
}

void Autonomy::moveFullAuto()
{
  // remote_reciever_.publishBreak();
}

void Autonomy::moveStop()
{
  // remote_reciever_.publishStop();
}

void Autonomy::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void Autonomy::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace autonomy

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomy::Autonomy, rviz::Panel)
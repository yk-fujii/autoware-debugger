/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>

#include "decision_maker_panel.h"
#include "drive_widget.h"

#include <state.hpp>
#include <state_context.hpp>

namespace autoware_rviz_debug
{
// BEGIN_TUTORIAL
// Here is the implementation of the DecisionMakerPanel class.  DecisionMakerPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.

class StateInfo
{
private:
public:
  uint64_t state_num_;
  uint8_t state_category_;
  std::string state_num_name_;
  std::string state_category_name_;
};

DecisionMakerPanel::DecisionMakerPanel(QWidget* parent) : rviz::Panel(parent), linear_velocity_(0), angular_velocity_(0)
{
  // Subs_["state"] = nh_
  state_machine::StateContext ctx;

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Output Topic:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  // std::unordered_map<uint64_t, state_machine::BaseState*> state_map = ctx.getStateStores();
  // std::map<std::string, uint8_t> state_kind_map = ctx.getStateKindMap();

  std::vector<StateInfo> states;
// name + id + category
#if 0
  for (auto it_state = std::map::begin(state_map); it_state != std::map::end(state_map); it_state++)
  {
    StateInfo _state;
    _state.state_num_ = it_state->getStateNum();
    _state.state_category_ = it_state->getStateKind();
    _state.state_num_name_ = it_state->getStateName();
    states.push_back(_state);
  }
#endif
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);
}

void DecisionMakerPanel::setVel(float lin, float ang)
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

void DecisionMakerPanel::updateTopic()
{
  setTopic(output_topic_editor_->text());
}

void DecisionMakerPanel::setTopic(const QString& new_topic)
{
  if (new_topic != output_topic_)
  {
    output_topic_ = new_topic;
    if (output_topic_ == "")
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_topic_.toStdString(), 1);
    }
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  drive_widget_->setEnabled(output_topic_ != "");
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void DecisionMakerPanel::sendVel()
{
  if (ros::ok() && velocity_publisher_)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish(msg);
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void DecisionMakerPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void DecisionMakerPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic))
  {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

}  // end namespace autoware_rviz_debug

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_debug::DecisionMakerPanel, rviz::Panel)
// END_TUTORIAL

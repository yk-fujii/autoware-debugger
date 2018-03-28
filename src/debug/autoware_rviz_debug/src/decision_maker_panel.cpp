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
#include <QPushButton>
#include <QSignalMapper>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>
#include "decision_maker_panel.h"

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

DecisionMakerPanel::DecisionMakerPanel(QWidget* parent) : rviz::Panel(parent)
{
  statecmd_publisher_ = nh_.advertise<std_msgs::Int32>("/state_cmd", 1);
  // Subs_["state"] = nh_

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* label_layout = new QHBoxLayout;
  label_layout->addWidget(new QLabel("DecisionMaker QuickOrder"));

  QSignalMapper* signalMapper = new QSignalMapper(this);
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(sendTopic(int)));

  QHBoxLayout* button_layout = new QHBoxLayout;
  QPushButton* button_stop = new QPushButton("Stop");
  // button_stop->setObjectName(QString::number(14));
  signalMapper->setMapping(button_stop, 14);
  connect(button_stop, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_start = new QPushButton("Start");
  // button_start->setObjectName(QString::number(13));
  signalMapper->setMapping(button_start, 13);
  connect(button_start, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_lanechange = new QPushButton("lanechange");
  //  button_lanechange->setObjectName(QString::number(37));
  signalMapper->setMapping(button_lanechange, 37);
  connect(button_lanechange, SIGNAL(clicked()), signalMapper, SLOT(map()));
  
  QPushButton* button_drive = new QPushButton("drive");
  //  button_lanechange->setObjectName(QString::number(37));
  signalMapper->setMapping(button_drive, 6);
  connect(button_drive, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_layout->addWidget(button_stop);
  button_layout->addWidget(button_start);
  button_layout->addWidget(button_lanechange);
  button_layout->addWidget(button_drive);
  
  QHBoxLayout* button_low_layout = new QHBoxLayout;
  QPushButton* button_green = new QPushButton("Green");
  signalMapper->setMapping(button_green, 35);
  connect(button_green, SIGNAL(clicked()), signalMapper, SLOT(map()));
  
  QPushButton* button_red = new QPushButton("Red");
  signalMapper->setMapping(button_red, 34);
  connect(button_red, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_low_layout->addWidget(button_green);
  button_low_layout->addWidget(button_red);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(label_layout);
  layout->addLayout(button_layout);
  layout->addLayout(button_low_layout);
  setLayout(layout);
}

void DecisionMakerPanel::sendTopic(int flag)
{
  if (statecmd_publisher_)
  {
    std_msgs::Int32 msg;
    msg.data = flag;
    statecmd_publisher_.publish(msg);
  }
}

void DecisionMakerPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void DecisionMakerPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}  // end namespace autoware_rviz_debug

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_debug::DecisionMakerPanel, rviz::Panel)
// END_TUTORIAL

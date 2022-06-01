/*
 *Copyright 2022, Tatsuhiro Ikebe.
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#ifndef RVIZ_WAYPOINT_POSE_TOOL_H
#define RVIZ_WAYPOINT_POSE_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
class FloatProperty;

class WaypointPoseTool : public PoseTool
{
  Q_OBJECT
 public:
  WaypointPoseTool();
  ~WaypointPoseTool() override {}
  void onInitialize() override;

 protected:
  void onPoseSet(double x, double y, double theta) override;

 private Q_SLOTS:
  void updateTopic();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  StringProperty* topic_property_;
  FloatProperty* std_dev_x_;
  FloatProperty* std_dev_y_;
  FloatProperty* std_dev_theta_;
};

}  // namespace rviz

#endif
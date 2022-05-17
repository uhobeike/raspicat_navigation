#ifndef RVIZ_WAYPOINT_POSE_TOOL_H
#define RVIZ_WAYPOINT_POSE_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
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
  ~WaypointPoseTool() override
  {
  }
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

} // namespace rviz

#endif
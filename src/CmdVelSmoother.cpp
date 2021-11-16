#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_navigation/WaypointNavHelperPlugin.hpp"

namespace raspicat_navigation
{
class CmdVelSmoother : public raspicat_navigation::WaypointNavHelperPlugin
{
 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::CmdVelSmoother initialize"); }
  void run() { ROS_INFO("raspicat_navigation::CmdVelSmoother run"); }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::CmdVelSmoother,
                       raspicat_navigation::WaypointNavHelperPlugin)
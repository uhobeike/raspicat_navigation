#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_navigation/BaseWaypointParamChange.hpp"

namespace raspicat_navigation
{
class WaypointParamChange : public raspicat_navigation::BaseWaypointParamChange
{
 public:
  void initialize(std::string name)
  {
    ROS_INFO("raspicat_navigation::WaypointParamChange initialize");
  }
  void run() { ROS_INFO("raspicat_navigation::WaypointParamChange run"); }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointParamChange,
                       raspicat_navigation::BaseWaypointParamChange)
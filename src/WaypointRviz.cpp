#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_navigation/BaseWaypointRviz.hpp"

namespace raspicat_navigation
{
class WaypointRviz : public raspicat_navigation::BaseWaypointRviz
{
 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::WaypointRviz initialize"); }
  void run() { ROS_INFO("raspicat_navigation::WaypointRviz run"); }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointRviz, raspicat_navigation::BaseWaypointRviz)
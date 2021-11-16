#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_navigation/BaseWaypointServer.hpp"

namespace raspicat_navigation
{
class WaypointServer : public raspicat_navigation::BaseWaypointServer
{
 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::WaypointServer initialize"); }
  void run() { ROS_INFO("raspicat_navigation::WaypointServer run"); }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
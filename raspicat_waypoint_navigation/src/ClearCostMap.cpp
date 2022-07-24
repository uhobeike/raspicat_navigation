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

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "raspicat_waypoint_navigation/ClearCostMap.hpp"

namespace raspicat_navigation
{
void ClearCostMap::initServiceServer()
{
  clear_cost_map_client_ = nh_.serviceClient<std_srvs::Empty::Request, std_srvs::Empty::Response>(
      "/move_base/clear_costmaps");
}

void ClearCostMap::clearCostMap()
{
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response resp;

  if (!clear_cost_map_client_.call(req, resp))
  {
    ROS_ERROR("Failed to invoke clear_costmaps services.");
  }
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::ClearCostMap,
                       raspicat_navigation::WaypointNavHelperPlugin)
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

#ifndef CLEAR_COST_MAP_HPP_
#define CLEAR_COST_MAP_HPP_

#include <ros/ros.h>

#include "raspicat_waypoint_navigation/WaypointNavHelperPlugin.hpp"

namespace raspicat_navigation
{
class ClearCostMap : public raspicat_navigation::WaypointNavHelperPlugin
{
  ros::NodeHandle nh_, pnh_;
  ros::ServiceClient clear_cost_map_client_;

 public:
  void initialize(std::string name)
  {
    ROS_INFO("raspicat_navigation::ClearCostMap initialize");
    initServiceServer();
  }
  void run()
  {
    ROS_INFO("raspicat_navigation::ClearCostMap run");
    clearCostMap();
  }

  void initServiceServer();

  void clearCostMap();
};

}  // namespace raspicat_navigation
#endif  // CLEAR_COST_MAP
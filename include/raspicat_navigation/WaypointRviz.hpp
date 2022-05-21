/*
 *Copyright 2021, Tatsuhiro Ikebe.
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

#ifndef WAYPOINT_RVIZ_HPP_
#define WAYPOINT_RVIZ_HPP_

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include "raspicat_navigation/BaseWaypointRviz.hpp"

namespace raspicat_navigation
{
class WaypointRviz : public raspicat_navigation::BaseWaypointRviz
{
 public:
  void initialize(std::string name);
  void run();
  void WaypointRvizVisualization(vector<vector<string>> &waypoint_csv_, int &waypoint_csv_index_,
                                 ros::Publisher &way_pose_array_, ros::Publisher &way_area_array_,
                                 ros::Publisher &way_number_txt_array_,
                                 float &waypoint_area_threshold_);

  void WaypointMarkerArraySet(visualization_msgs::MarkerArray &waypoint_area,
                              visualization_msgs::MarkerArray &waypoint_number_txt, uint8_t index,
                              uint8_t size, float &waypoint_area_threshold_,
                              vector<vector<string>> &waypoint_csv_);

  virtual ~WaypointRviz() {}
};

}  // namespace raspicat_navigation
#endif  // WAYPOINT_RVIZ_HPP_
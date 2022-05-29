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

#ifndef WAYPOINT_SERVER_HPP_
#define WAYPOINT_SERVER_HPP_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <sstream>

#include "raspicat_navigation/BaseWaypointServer.hpp"

namespace raspicat_navigation
{
class WaypointServer : public raspicat_navigation::BaseWaypointServer
{
 public:
  void initialize(std::string name);
  void run();
  void WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
                       int &waypoint_csv_index);

  void checkWaypointYmal(ros::NodeHandle &pnh_);

  void loadWaypointYmal(ros::NodeHandle &pnh, XmlRpc::XmlRpcValue &waypoint_yaml);

  void setWaypoint(move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_,
                   int &waypoint_index_,
                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_);

  bool checkWaypointArea(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                         vector<vector<string>> &waypoint_csv, int &waypoint_index,
                         ros::Publisher &way_passed);

  void getRobotPose(tf2_ros::Buffer &tf_,
                    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus);

  bool checkGoalReach(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                      int &waypoint_index_);
  void ModeFlagOff(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus);

  void managementWaypointInfo(vector<vector<string>> &waypoint_csv, int &waypoint_csv_index,
                              int &waypoint_index,
                              raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                              ros::Publisher &way_passed, ros::Publisher &way_finish,
                              ros::Publisher &way_mode_slope);

  void debug(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus, int &waypoint_index);

  virtual ~WaypointServer() {}
};
}  // namespace raspicat_navigation
#endif  // WAYPOINT_SERVER_HPP_
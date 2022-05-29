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

#ifndef BASE_WAYPOINT_HELPER_PLUGIN_HPP_
#define BASE_WAYPOINT_HELPER_PLUGIN_HPP_

using namespace ::std;

#include "raspicat_navigation_msgs/WaypointNavStatus.h"

namespace raspicat_navigation
{
class BaseWaypointServer
{
 public:
  virtual void initialize(std::string name) = 0;
  virtual void run() = 0;
  virtual void WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
                               int &waypoint_csv_index_) = 0;

  virtual void checkWaypointYmal(ros::NodeHandle &pnh) = 0;
  virtual void loadWaypointYmal(ros::NodeHandle &pnh, XmlRpc::XmlRpcValue &waypoint_yaml) = 0;

  virtual void setWaypoint(
      move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_,
      int &waypoint_index_,
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_) = 0;

  virtual bool checkWaypointArea(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                                 vector<vector<string>> &waypoint_csv_, int &waypoint_index_,
                                 ros::Publisher &way_passed_) = 0;

  virtual bool checkGoalReach(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                              int &waypoint_index_) = 0;

  virtual void getRobotPose(tf2_ros::Buffer &tf_,
                            raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus) = 0;

  virtual void ModeFlagOff(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus) = 0;

  virtual void managementWaypointInfo(
      vector<vector<string>> &waypoint_csv_, int &waypoint_csv_index_, int &waypoint_index_,
      raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus, ros::Publisher &way_passed_,
      ros::Publisher &way_finish, ros::Publisher &way_mode_slope) = 0;

  virtual void debug(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                     int &waypoint_index) = 0;

  virtual ~BaseWaypointServer() {}

 protected:
  BaseWaypointServer() {}
};

}  // namespace raspicat_navigation
#endif  // BASE_WAYPOINT_HELPER_PLUGIN_HPP_
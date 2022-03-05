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

#include <ros/ros.h>

using namespace ::std;

namespace raspicat_navigation
{
class BaseWaypointServer
{
 public:
  virtual void initialize(std::string name) = 0;
  virtual void run() = 0;
  virtual void WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
                               int &waypoint_csv_index_) = 0;
  virtual void setWaypoint(
      move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_,
      int &waypoint_index_,
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_) = 0;

  virtual bool checkWaypointArea(bool &NextWaypointMode_, bool &SlopeObstacleAvoidanceMode_,
                                 float &waypoint_area_check_, vector<vector<string>> &waypoint_csv_,
                                 int &waypoint_index_, vector<double> &robot_pose_,
                                 float &waypoint_area_threshold_, string &node_name_,
                                 ros::Publisher &way_sound_) = 0;

  virtual bool checkGoalReach(bool &GoalReachedFlag_, string &node_name_, int &waypoint_index_) = 0;

  virtual void getRobotPose(tf2_ros::Buffer &tf_, vector<double> &robot_pose_) = 0;

  virtual void ModeFlagOff(bool &NextWaypointMode, bool &FinalGoalWaypointMode,
                           bool &ReStartWaypointMode, bool &GoalReachedMode,
                           bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag,
                           bool &GoalReachedFlag) = 0;

  virtual void managementWaypointInfo(vector<vector<string>> &waypoint_csv_,
                                      int &waypoint_csv_index_, int &waypoint_index_,
                                      string &node_name_, bool &NextWaypointMode_,
                                      bool &FinalGoalWaypointMode_, bool &ReStartWaypointMode_,
                                      bool &GoalReachedMode_, bool &GoalReachedFlag_,
                                      bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag_,
                                      bool &FinalGoalFlag_, float &waypoint_area_check_,
                                      vector<double> &robot_pose_, float &waypoint_area_threshold_,
                                      ros::Publisher &way_sound_) = 0;

  virtual void debug(bool &NextWaypointMode, bool &FinalGoalWaypointMode, bool &ReStartWaypointMode,
                     bool &GoalReachedMode, bool &GoalReachedFlag, bool &SlopeObstacleAvoidanceMode,
                     bool &SlopeObstacleAvoidanceFlag, int &waypoint_index,
                     float &waypoint_area_check, float &waypoint_area_threshold) = 0;

  virtual ~BaseWaypointServer() {}

 protected:
  BaseWaypointServer() {}
};

}  // namespace raspicat_navigation
#endif  // BASE_WAYPOINT_HELPER_PLUGIN_HPP_
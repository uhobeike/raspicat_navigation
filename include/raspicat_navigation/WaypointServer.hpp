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

  void setWaypoint(move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_,
                   int &waypoint_index_,
                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_);

  bool checkWaypointArea(bool &NextWaypointMode, bool &SlopeObstacleAvoidanceMode,
                         float &waypoint_area_check, vector<vector<string>> &waypoint_csv,
                         int &waypoint_index, vector<double> &robot_pose,
                         float &waypoint_area_threshold, string &node_name,
                         ros::Publisher &way_sound);

  void getRobotPose(tf2_ros::Buffer &tf_, vector<double> &robot_pose_);

  bool checkGoalReach(bool &GoalReachedFlag_, string &node_name_, int &waypoint_index_);
  void ModeFlagOff(bool &NextWaypointMode, bool &FinalGoalWaypointMode, bool &ReStartWaypointMode,
                   bool &GoalReachedMode, bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag,
                   bool &GoalReachedFlag);

  void managementWaypointInfo(vector<vector<string>> &waypoint_csv, int &waypoint_csv_index,
                              int &waypoint_index, string &node_name, bool &NextWaypointMode,
                              bool &FinalGoalWaypointMode, bool &ReStartWaypointMode,
                              bool &GoalReachedMode, bool &GoalReachedFlag,
                              bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag,
                              bool &FinalGoalFlag, float &waypoint_area_check,
                              vector<double> &robot_pose, float &waypoint_area_threshold,
                              ros::Publisher &way_sound, ros::Publisher &way_finish,
                              ros::Publisher &way_mode_slope);

  void debug(bool &NextWaypointMode, bool &FinalGoalWaypointMode, bool &ReStartWaypointMode,
             bool &GoalReachedMode, bool &GoalReachedFlag, bool &SlopeObstacleAvoidanceMode,
             bool &SlopeObstacleAvoidanceFlag, int &waypoint_index, float &waypoint_area_check,
             float &waypoint_area_threshold);

  virtual ~WaypointServer() {}
};
}  // namespace raspicat_navigation
#endif  // WAYPOINT_SERVER_HPP_
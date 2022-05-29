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

#ifndef WAYPOINT_NAV_
#define WAYPOINT_NAV_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_loader.hpp>
#include "raspicat_navigation/BaseWaypointRviz.hpp"
#include "raspicat_navigation/BaseWaypointServer.hpp"
#include "raspicat_navigation/WaypointNavHelperPlugin.hpp"
#include "raspicat_navigation_msgs/WaypointNavStatus.h"

#include <vector>

using namespace ::std;

namespace waypoint_nav
{
class WaypointNav
{
 public:
  WaypointNav(ros::NodeHandle& nodeHandle, ros::NodeHandle& private_nodeHandle, std::string name,
              std::string file_name, tf2_ros::Buffer& tf);
  virtual ~WaypointNav();

  void readParam();
  void initTimerCb();
  void initPubSub();
  void initActionClient();
  void initServiceClient();
  void initClassLoader();

  void Run();

  void GoalReachedCb(const actionlib_msgs::GoalStatusArray& status);
  void WaypointStartCb(const std_msgs::String& msg);
  void WaypointRestartCb(const std_msgs::String& msg);

 private:
  ros::NodeHandle &nh_, &pnh_;
  tf2_ros::Buffer& tf_;
  ros::Timer timer_;

  ros::Subscriber sub_robot_pose_, sub_movebase_goal_, sub_goal_command_, way_start_, way_restart_;
  ros::Publisher ini_pose_, way_pose_array_, way_area_array_, way_number_txt_array_, way_passed_,
      way_mode_slope_, way_finish_;

  std::map<std::string, ros::ServiceClient> slope_obstacle_avoidanc_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointServer> waypoint_server_loader_;
  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointRviz> waypoint_rviz_loader_;
  pluginlib::ClassLoader<raspicat_navigation::WaypointNavHelperPlugin> waypoint_nav_helper_loader_;
  boost::shared_ptr<raspicat_navigation::BaseWaypointServer> way_srv_;
  boost::shared_ptr<raspicat_navigation::BaseWaypointRviz> way_rviz_;
  boost::shared_ptr<raspicat_navigation::WaypointNavHelperPlugin> way_helper_;

  string waypoint_server_, waypoint_rviz_, waypoint_nav_helper_, node_name_;

  string csv_fname_;
  int waypoint_csv_index_;
  int waypoint_index_;
  vector<vector<string>> waypoint_csv_;
  vector<double> robot_pose_;

  float waypoint_area_threshold_;
  float waypoint_area_check_;

  move_base_msgs::MoveBaseGoal goal_;

  raspicat_navigation_msgs::WaypointNavStatus WaypointNavStatus_;

  bool MsgReceiveFlag_;
};

}  // namespace waypoint_nav
#endif
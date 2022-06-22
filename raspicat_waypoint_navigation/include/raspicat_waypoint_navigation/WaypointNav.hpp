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

#ifndef WAYPOINT_NAV_
#define WAYPOINT_NAV_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <dwa_local_planner/DWAPlannerConfig.h>
#include <dynamic_reconfigure/client.h>

#include <pluginlib/class_loader.hpp>
#include "raspicat_waypoint_navigation/BaseWaypointRviz.hpp"
#include "raspicat_waypoint_navigation/BaseWaypointServer.hpp"
#include "raspicat_waypoint_navigation/WaypointNavHelperPlugin.hpp"

#include <vector>

using namespace ::std;

namespace waypoint_nav
{
class WaypointNav
{
 public:
  WaypointNav(ros::NodeHandle& nodeHandle, ros::NodeHandle& private_nodeHandle,
              tf2_ros::Buffer& tf);
  virtual ~WaypointNav();

  void readParam();
  void getRbotPoseTimer();
  void initPubSub();
  void initActionClient();
  void initServiceClient();
  void initClassLoader();

  void resolve_tf_between_map_and_robot_link();

  void Run();

  void GoalReachedCb(const actionlib_msgs::GoalStatusArrayConstPtr& status);
  void WaypointNavStartCb(const std_msgs::EmptyConstPtr& msg);
  void WaypointNavRestartCb(const std_msgs::EmptyConstPtr& msg);

  void sleep(ros::Duration duration);
  void sleep(double rate);
  void sleep(double& rate);

 private:
  ros::NodeHandle &nh_, &pnh_;
  tf2_ros::Buffer& tf_;
  ros::Timer resolve_tf_timer_, get_robot_pose_timer_;
  std::map<std::string, ros::Timer> timer_for_function_;

  ros::Subscriber sub_robot_pose_, sub_movebase_goal_, sub_goal_command_, sub_way_start_,
      sub_way_restart_;
  ros::Publisher way_pose_array_, way_area_array_, way_number_txt_array_, way_passed_, way_stop_,
      way_slope_, way_goal_, way_loop_, way_attention_speak_;

  std::map<std::string, ros::ServiceClient> slope_obstacle_avoidanc_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointServer> waypoint_server_loader_;
  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointRviz> waypoint_rviz_loader_;
  pluginlib::ClassLoader<raspicat_navigation::WaypointNavHelperPlugin> waypoint_nav_helper_loader_;
  boost::shared_ptr<raspicat_navigation::BaseWaypointServer> way_srv_;
  boost::shared_ptr<raspicat_navigation::BaseWaypointRviz> way_rviz_;
  boost::shared_ptr<raspicat_navigation::WaypointNavHelperPlugin> way_helper_;

  string waypoint_server_, waypoint_rviz_, waypoint_nav_helper_;

  XmlRpc::XmlRpcValue waypoint_yaml_;

  move_base_msgs::MoveBaseGoal goal_;

  raspicat_navigation_msgs::WaypointNavStatus WaypointNavStatus_;

  double waypoint_radius_;
  double vel_trans_;
  dynamic_reconfigure::Client<dwa_local_planner::DWAPlannerConfig> dynamic_reconfigure_client_;
};

}  // namespace waypoint_nav
#endif
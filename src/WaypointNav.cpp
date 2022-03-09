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

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "raspicat_navigation/WaypointNav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace ::std;

namespace waypoint_nav
{
WaypointNav::WaypointNav(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nodeHandle,
                         std::string node_name, std::string file_name, tf2_ros::Buffer &tf)
    : nh_(nodeHandle),
      pnh_(private_nodeHandle),
      tf_(tf),
      ac_move_base_("move_base", true),
      waypoint_server_loader_("raspicat_navigation", "raspicat_navigation::BaseWaypointServer"),
      waypoint_rviz_loader_("raspicat_navigation", "raspicat_navigation::BaseWaypointRviz"),
      waypoint_nav_helper_loader_("raspicat_navigation",
                                  "raspicat_navigation::WaypointNavHelperPlugin"),
      node_name_(node_name),
      csv_fname_(file_name),
      waypoint_csv_index_(1),
      waypoint_index_(0),
      waypoint_csv_(1, vector<string>(0)),
      robot_pose_(2, 0),
      waypoint_area_threshold_(5.0),
      waypoint_area_check_(0.0),
      NextWaypointMode_(true),
      FinalGoalWaypointMode_(false),
      ReStartWaypointMode_(false),
      GoalReachedMode_(false),
      GoalReachedFlag_(false),
      SlopeObstacleAvoidanceMode_(false),
      SlopeObstacleAvoidanceFlag_(false),
      FinalGoalFlag_(false),
      ReStartFlag_(false),
      MsgReceiveFlag_(false)
{
  readParam();
  initActionClient();
  initPubSub();
  initClassLoader();
  initTimerCb();
  initServiceClient();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::readParam()
{
  pnh_.param("base_waypoint_server", waypoint_server_,
             std::string("raspicat_navigation/WaypointServer"));

  pnh_.param("base_waypoint_rviz", waypoint_rviz_, std::string("raspicat_navigation/WaypointRviz"));

  pnh_.param("waypoint_nav_helper", waypoint_nav_helper_,
             std::string("raspicat_navigation/CmdVelSmoother"));
}

void WaypointNav::initTimerCb()
{
  ros::Duration duration(1.0);
  duration.sleep();
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           [&](auto &) { way_srv_->getRobotPose(tf_, robot_pose_); });
}

void WaypointNav::initPubSub()
{
  sub_movebase_goal_ = nh_.subscribe("move_base/status", 1, &WaypointNav::GoalReachedCb, this);
  sub_goal_command_ = nh_.subscribe("goal_command", 1, &WaypointNav::GoalCommandCb, this);
  waypoint_start_ = nh_.subscribe("waypoint_start", 1, &WaypointNav::WaypointStartCb, this);
  waypoint_restart_ = nh_.subscribe("waypoint_restart", 1, &WaypointNav::WaypointRestartCb, this);

  way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
  way_area_array_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_area", 1, true);
  way_sound_ = nh_.advertise<std_msgs::Bool>("waypoint_passed", 1, true);
  way_number_txt_array_ =
      nh_.advertise<visualization_msgs::MarkerArray>("waypoint_number_txt", 1, true);
}

void WaypointNav::initActionClient()
{
  ROS_INFO("Waiting for move_base Action Server to active.");
  while (!ac_move_base_.waitForServer(ros::Duration(100.0)))
  {
    ROS_ERROR("move_base Action Server is not active.");
    exit(0);
  }
  ROS_INFO("move_base Action Server is active.");
}

void WaypointNav::initServiceClient()
{
  slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_on"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "slope_obstacle_avoidance_on");

  ROS_INFO("Waiting for SlopeObstacleAvoidance Server to active.");
  if (!slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_on"].waitForExistence(
          ros::Duration(100.0)))
  {
    ROS_ERROR("SlopeObstacleAvoidance Server is not active.");
    exit(0);
  }
  ROS_INFO("SlopeObstacleAvoidance Server is active.");

  slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_off"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "slope_obstacle_avoidance_off");

  ROS_INFO("Waiting for SlopeObstacleAvoidance Server to active.");
  if (!slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_off"].waitForExistence(
          ros::Duration(100.0)))
  {
    ROS_ERROR("SlopeObstacleAvoidance Server is not active.");
    exit(0);
  }
  ROS_INFO("SlopeObstacleAvoidance Server is active.");
}

void WaypointNav::initClassLoader()
{
  try
  {
    way_srv_ = waypoint_server_loader_.createInstance("raspicat_navigation/WaypointServer");
    way_srv_->initialize(waypoint_server_);
    way_srv_->run();
    way_srv_->WaypointCsvRead(csv_fname_, waypoint_csv_, waypoint_csv_index_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    way_rviz_ = waypoint_rviz_loader_.createInstance("raspicat_navigation/WaypointRviz");
    way_rviz_->initialize(waypoint_rviz_);
    way_rviz_->run();
    way_rviz_->WaypointRvizVisualization(waypoint_csv_, waypoint_csv_index_, way_pose_array_,
                                         way_area_array_, way_number_txt_array_,
                                         waypoint_area_threshold_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  // try
  // {
  //   way_helper_ =
  //   waypoint_nav_helper_loader_.createInstance("raspicat_navigation/CmdVelSmoother");
  //   way_helper_->initialize(waypoint_nav_helper_);
  //   way_helper_->run();
  // }
  // catch (pluginlib::PluginlibException &ex)
  // {
  //   ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  // }

  try
  {
    way_helper_ =
        waypoint_nav_helper_loader_.createInstance("raspicat_navigation/SlopeObstacleAvoidance");
    way_helper_->initialize(waypoint_nav_helper_);
    way_helper_->run();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
}

void WaypointNav::Run()
{
  way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if (NextWaypointMode_)
    {
      if (way_srv_->checkWaypointArea(
              NextWaypointMode_, SlopeObstacleAvoidanceMode_, waypoint_area_check_, waypoint_csv_,
              waypoint_index_, robot_pose_, waypoint_area_threshold_, node_name_, way_sound_))
        way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);
    }
    else if (FinalGoalWaypointMode_)
      way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);
    else if (ReStartWaypointMode_)
    {
      if (ReStartFlag_) way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);
    }
    else if (GoalReachedMode_)
    {
      if (way_srv_->checkWaypointArea(
              NextWaypointMode_, SlopeObstacleAvoidanceMode_, waypoint_area_check_, waypoint_csv_,
              waypoint_index_, robot_pose_, waypoint_area_threshold_, node_name_, way_sound_) &&
          way_srv_->checkGoalReach(GoalReachedFlag_, node_name_, waypoint_index_))
        way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);
    }
    else if (SlopeObstacleAvoidanceMode_)
    {
      if (!SlopeObstacleAvoidanceFlag_)
      {
        std_srvs::TriggerRequest req;
        std_srvs::TriggerResponse resp;
        if (!slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_on"].call(req, resp))
        {
          ROS_ERROR("Failed to invoke slope_obstacle_avoidance_on services.");
          exit(0);
        }
        SlopeObstacleAvoidanceFlag_ = true;
      }
      if (way_srv_->checkWaypointArea(
              NextWaypointMode_, SlopeObstacleAvoidanceMode_, waypoint_area_check_, waypoint_csv_,
              waypoint_index_, robot_pose_, waypoint_area_threshold_, node_name_, way_sound_))
      {
        way_srv_->setWaypoint(goal_, waypoint_csv_, waypoint_index_, ac_move_base_);
        std_srvs::TriggerRequest req;
        std_srvs::TriggerResponse resp;
        if (!slope_obstacle_avoidanc_client_["slope_obstacle_avoidance_off"].call(req, resp))
        {
          ROS_ERROR("Failed to invoke slope_obstacle_avoidance_off services.");
          exit(0);
        }
        SlopeObstacleAvoidanceFlag_ = false;
      }
    }
    way_srv_->debug(NextWaypointMode_, FinalGoalWaypointMode_, ReStartWaypointMode_,
                    GoalReachedMode_, GoalReachedFlag_, SlopeObstacleAvoidanceMode_,
                    SlopeObstacleAvoidanceFlag_, waypoint_index_, waypoint_area_check_,
                    waypoint_area_threshold_);

    way_srv_->managementWaypointInfo(
        waypoint_csv_, waypoint_csv_index_, waypoint_index_, node_name_, NextWaypointMode_,
        FinalGoalWaypointMode_, ReStartWaypointMode_, GoalReachedMode_, GoalReachedFlag_,
        SlopeObstacleAvoidanceMode_, ReStartFlag_, FinalGoalFlag_, waypoint_area_check_,
        robot_pose_, waypoint_area_threshold_, way_sound_);
    ros::spinOnce();
    loop_rate.sleep();
  }
}  // namespace waypoint_nav

void WaypointNav::GoalReachedCb(const actionlib_msgs::GoalStatusArray &status)
{
  if (!status.status_list.empty())
  {
    actionlib_msgs::GoalStatus goalStatus = status.status_list[0];

    if (goalStatus.status == 3 && GoalReachedFlag_ == false) GoalReachedFlag_ = true;
  }
}

void WaypointNav::GoalCommandCb(const std_msgs::String &msg)
{
  if (msg.data == "go" && !MsgReceiveFlag_)
  {
    MsgReceiveFlag_ = true;
    Run();
  }
  else if (msg.data == "go" && MsgReceiveFlag_)
  {
    ReStartFlag_ = true;
    waypoint_index_++;
  }
  else if (msg.data == "q" && MsgReceiveFlag_)
  {
    ROS_INFO("%s: Shutdown now ('o')/ bye bye~~~", node_name_.c_str());
    ros::shutdown();
  }
}

void WaypointNav::WaypointStartCb(const std_msgs::String &msg)
{
  if (!MsgReceiveFlag_)
  {
    MsgReceiveFlag_ = true;
    Run();
  }
}

void WaypointNav::WaypointRestartCb(const std_msgs::String &msg)
{
  ReStartFlag_ = true;
  waypoint_index_++;
}

}  // namespace waypoint_nav
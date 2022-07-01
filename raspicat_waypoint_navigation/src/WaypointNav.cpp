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

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"

#include "raspicat_waypoint_navigation/WaypointNav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace ::std;

namespace waypoint_nav
{
WaypointNav::WaypointNav(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nodeHandle,
                         tf2_ros::Buffer &tf)
    : nh_(nodeHandle),
      pnh_(private_nodeHandle),
      tf_(tf),
      ac_move_base_("move_base", true),
      dynamic_reconfigure_client_("/move_base/DWAPlannerROS"),
      waypoint_server_loader_("raspicat_waypoint_navigation",
                              "raspicat_navigation::BaseWaypointServer"),
      waypoint_rviz_loader_("raspicat_waypoint_navigation",
                            "raspicat_navigation::BaseWaypointRviz"),
      waypoint_nav_helper_loader_("raspicat_waypoint_navigation",
                                  "raspicat_navigation::WaypointNavHelperPlugin"),
      waypoint_radius_(3.0)
{
  readParam();
  initPub();
  initActionClient();
  initSub();
  initClassLoader();
  getRobotPoseTimer();
  initServiceClient();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::readParam()
{
  pnh_.getParam("/move_base/DWAPlannerROS/max_vel_trans", vel_trans_);
}

void WaypointNav::getRobotPoseTimer()
{
  sleep(1.0);
  get_robot_pose_timer_ = nh_.createTimer(
      ros::Duration(0.1), [&](auto &) { way_srv_->getRobotPose(tf_, WaypointNavStatus_); });
}

void WaypointNav::resolve_tf_between_map_and_robot_link()
{
  sleep(1.0);
  resolve_tf_timer_ = nh_.createTimer(ros::Duration(1.0), [&](auto &) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    double angle_z;
    pnh_.getParam("/WaypointNav_node/initial_pose_x", msg.pose.pose.position.x);
    pnh_.getParam("/WaypointNav_node/initial_pose_y", msg.pose.pose.position.y);
    pnh_.getParam("/WaypointNav_node/initial_pose_a", angle_z);

    tf2::Quaternion q;
    q.setRPY(0, 0, static_cast<double>(angle_z));
    msg.pose.pose.orientation.z = q.getZ();
    msg.pose.pose.orientation.w = q.getW();
    msg.pose.covariance.at(0) = 0.25;
    msg.pose.covariance.at(7) = 0.25;
    msg.pose.covariance.at(35) = 0.06853892326654787;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    mcl_init_pose_.publish(msg);
  });
}

void WaypointNav::initPub()
{
  mcl_init_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

  way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/waypoint", 1, true);
  way_area_array_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_area", 1, true);
  way_number_txt_array_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_number_txt", 1, true);

  way_passed_ = nh_.advertise<std_msgs::Empty>("/waypoint_passed", 1, true);
  way_stop_ = nh_.advertise<std_msgs::Empty>("/waypoint_stop_function", 1, true);
  way_slope_ = nh_.advertise<std_msgs::Empty>("/waypoint_slope_function", 1, true);
  way_goal_ = nh_.advertise<std_msgs::Empty>("/waypoint_goal_function", 1, true);
  way_loop_ = nh_.advertise<std_msgs::Empty>("/waypoint_loop_function", 1, true);
  way_attention_speak_ =
      nh_.advertise<std_msgs::Empty>("/waypoint_attention_speak_function", 1, true);
  way_clear_costmap_ = nh_.advertise<std_msgs::Empty>("/waypoint_clear_costmap_function", 1, true);
}

void WaypointNav::initSub()
{
  sub_movebase_goal_ = nh_.subscribe("/move_base/status", 1, &WaypointNav::GoalReachedCb, this);
  sub_way_start_ = nh_.subscribe("/way_nav_start", 1, &WaypointNav::WaypointNavStartCb, this);
  sub_way_restart_ = nh_.subscribe("/way_nav_restart", 1, &WaypointNav::WaypointNavRestartCb, this);
}

void WaypointNav::initActionClient()
{
  ROS_INFO("Waiting for move_base Action Server to active.");
  resolve_tf_between_map_and_robot_link();
  sleep(3.0);
  ros::spinOnce();
  while (!ac_move_base_.waitForServer(ros::Duration(120.0)))
  {
    ROS_ERROR("move_base Action Server is not active.");
    exit(0);
  }
  resolve_tf_timer_.stop();
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
    way_srv_ =
        waypoint_server_loader_.createInstance("raspicat_waypoint_navigation/WaypointServer");
    way_srv_->initialize(waypoint_server_);
    way_srv_->run();
    // way_srv_->checkWaypointYmal(pnh_);
    way_srv_->loadWaypointYmal(pnh_, waypoint_yaml_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    way_rviz_ = waypoint_rviz_loader_.createInstance("raspicat_waypoint_navigation/WaypointRviz");
    way_rviz_->initialize(waypoint_rviz_);
    way_rviz_->run();
    WaypointNavStatus_.waypoint_radius_threshold = waypoint_radius_;
    way_rviz_->WaypointRvizVisualization(waypoint_yaml_, way_pose_array_, way_area_array_,
                                         way_number_txt_array_,
                                         WaypointNavStatus_.waypoint_radius_threshold);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    way_helper_["SlopeObstacleAvoidance"] = waypoint_nav_helper_loader_.createInstance(
        "raspicat_waypoint_navigation/SlopeObstacleAvoidance");
    way_helper_["SlopeObstacleAvoidance"]->initialize(waypoint_nav_helper_);
    way_helper_["SlopeObstacleAvoidance"]->run();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    way_helper_["ClearCostMap"] =
        waypoint_nav_helper_loader_.createInstance("raspicat_waypoint_navigation/ClearCostMap");
    way_helper_["ClearCostMap"]->initialize(waypoint_nav_helper_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
}

void WaypointNav::Run()
{
  way_srv_->setWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
  way_srv_->setFalseWaypointFunction(WaypointNavStatus_);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    WaypointNavStatus_.waypoint_previous_id = WaypointNavStatus_.waypoint_current_id;

    // set function
    way_srv_->setWaypointFunction(dynamic_reconfigure_client_, waypoint_yaml_, WaypointNavStatus_);

    // next_waypoint function
    if (WaypointNavStatus_.functions.next_waypoint.function)
      if (way_srv_->checkWaypointArea(waypoint_yaml_, WaypointNavStatus_, way_passed_))
        way_srv_->setNextWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);

    // stop function
    if (WaypointNavStatus_.functions.stop.function)
    {
      if (way_srv_->checkGoalReach(WaypointNavStatus_))
        if (WaypointNavStatus_.flags.restart)
        {
          timer_for_function_.erase("speak_stop");
          way_srv_->setNextWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
          way_srv_->setFalseWaypointFlag(WaypointNavStatus_, true);
        }
        else
        {
          if (timer_for_function_.find("speak_stop") == timer_for_function_.end())
          {
            ros::Timer speak_stop;
            speak_stop = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
              std_msgs::Empty msg;
              way_stop_.publish(msg);
              sleep(6.0);
            });
            timer_for_function_["speak_stop"] = speak_stop;
          }
        }
    }

    // goal function
    if (WaypointNavStatus_.functions.goal.function)
      if (way_srv_->checkGoalReach(WaypointNavStatus_))
      {
        ROS_INFO("Waypoint Navigation Finish!");
        if (timer_for_function_.find("speak_goal") == timer_for_function_.end())
        {
          ros::Timer speak_goal;
          speak_goal = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
            std_msgs::Empty msg;
            way_goal_.publish(msg);
            sleep(5.0);
          });
          timer_for_function_["speak_goal"] = speak_goal;
        }
      }

    // loop function
    if (WaypointNavStatus_.functions.loop.function)
      if (way_srv_->checkGoalReach(WaypointNavStatus_))
      {
        sleep(5.0);
        std_msgs::Empty msg;
        way_loop_.publish(msg);
        ROS_INFO("Waypoint Navigation Loop!");
        WaypointNavStatus_.waypoint_current_id = 0;
        way_srv_->setWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
        timer_for_function_.erase(timer_for_function_.begin(), timer_for_function_.end());
      }

    // attention speak function
    if (WaypointNavStatus_.functions.attention_speak.function)
      if (timer_for_function_.find("speak_attention") == timer_for_function_.end())
      {
        ros::Timer speak_attention;
        speak_attention = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
          std_msgs::Empty msg;
          way_attention_speak_.publish(msg);
          sleep(5.0);
        });
        timer_for_function_["speak_attention"] = speak_attention;
      }

    // variable speed function
    if (not WaypointNavStatus_.functions.variable_speed.function)
    {
      dwa_local_planner::DWAPlannerConfig config;
      dynamic_reconfigure_client_.getCurrentConfiguration(config);
      config.max_vel_trans = vel_trans_;
      config.max_vel_x = vel_trans_;
      dynamic_reconfigure_client_.setConfiguration(config);
    }

    // variable waypoint radius function
    if (not WaypointNavStatus_.functions.variable_waypoint_radius.function)
      WaypointNavStatus_.waypoint_radius_threshold = waypoint_radius_;

    // Call /move_base/clear_costmap service when waypoint is reached.
    if ((WaypointNavStatus_.waypoint_previous_id != WaypointNavStatus_.waypoint_current_id) &&
        WaypointNavStatus_.functions.clear_costmap.function)
    {
      ac_move_base_.cancelAllGoals();
      sleep(1);
      way_helper_["ClearCostMap"]->run();
      sleep(5);
      std_msgs::Empty msg;
      way_clear_costmap_.publish(msg);
      sleep(5);
      way_srv_->sendWaypoint(ac_move_base_, goal_);
    }

    way_srv_->debug(WaypointNavStatus_);
    way_srv_->eraseTimer(WaypointNavStatus_, timer_for_function_);
    way_srv_->setFalseWaypointFunction(WaypointNavStatus_);
    way_srv_->setFalseWaypointFlag(WaypointNavStatus_);
    ros::spinOnce();
    loop_rate.sleep();
  }
}  // namespace waypoint_nav

void WaypointNav::WaypointNavStartCb(const std_msgs::EmptyConstPtr &msg)
{
  static bool call_once = true;
  if (call_once)
  {
    call_once = false;
    Run();
  }
}

void WaypointNav::GoalReachedCb(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
  if (!status->status_list.empty())
  {
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];

    if (goalStatus.status == 3 && WaypointNavStatus_.flags.goal_reach == false)
      WaypointNavStatus_.flags.goal_reach = true;
  }
}

void WaypointNav::WaypointNavRestartCb(const std_msgs::EmptyConstPtr &msg)
{
  WaypointNavStatus_.flags.restart = true;
}

void WaypointNav::sleep(double time_s) { ros::Duration(time_s).sleep(); }

void WaypointNav::sleep(double &time_s) { ros::Duration(time_s).sleep(); }

}  // namespace waypoint_nav
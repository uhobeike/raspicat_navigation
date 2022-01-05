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

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_loader.hpp>
#include "raspicat_navigation/BaseWaypointRviz.hpp"
#include "raspicat_navigation/BaseWaypointServer.hpp"
#include "raspicat_navigation/WaypointNav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace ::std;

namespace waypoint_nav
{
WaypointNav::WaypointNav(ros::NodeHandle& nodeHandle, std::string node_name, std::string file_name,
                         tf2_ros::Buffer& tf)
    : nh_(nodeHandle),
      tf_(tf),
      ac_move_base_("move_base", true),
      node_name_(node_name),
      csv_fname_(file_name),
      waypoint_csv_index_(1),
      waypoint_index_(0),
      waypoint_csv_(1, vector<string>(0)),
      robot_pose_(2, 0),
      waypoint_area_threshold_(1.5),
      waypoint_area_check_(0.0),
      NextWaypointMode_(true),
      FinalGoalWaypointMode_(false),
      ReStartWaypointMode_(false),
      GoalReachedMode_(false),
      GoalReachedFlag_(false),
      FinalGoalFlag_(false),
      ReStartFlag_(false),
      MsgReceiveFlag_(false)
{
  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointServer> base_waypoint_server_loader(
      "raspicat_navigation", "raspicat_navigation::BaseWaypointServer");

  pluginlib::ClassLoader<raspicat_navigation::BaseWaypointRviz> base_waypoint_rviz_loader(
      "raspicat_navigation", "raspicat_navigation::BaseWaypointRviz");

  std::string plugin_name1 = "raspicat_navigation/WaypointServer";
  std::string plugin_name2 = "raspicat_navigation/WaypointRviz";

  try
  {
    boost::shared_ptr<raspicat_navigation::BaseWaypointServer> add =
        base_waypoint_server_loader.createInstance("raspicat_navigation/WaypointServer");
    add->initialize(plugin_name1);
    add->run();
    add->WaypointCsvRead(csv_fname_, waypoint_csv_, waypoint_csv_index_);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    boost::shared_ptr<raspicat_navigation::BaseWaypointRviz> add =
        base_waypoint_rviz_loader.createInstance("raspicat_navigation/WaypointRviz");
    add->initialize(plugin_name2);
    add->run();
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
  initTimerCb();

  PubSub_Init();
  ActionClient_Init();

  // WaypointCsvRead();
  WaypointRvizVisualization();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::initTimerCb()
{
  timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointNav::getRobotPose, this);
}

void WaypointNav::PubSub_Init()
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

void WaypointNav::ActionClient_Init()
{
  while (!ac_move_base_.waitForServer(ros::Duration(100.0)))
  {
    ROS_ERROR("Waiting for the move_base action server to come up");
    exit(0);
  }
  ROS_INFO("MoveBase server comes up");
}

void WaypointNav::WaypointRvizVisualization()
{
  geometry_msgs::PoseArray pose_array;
  geometry_msgs::Pose pose;
  visualization_msgs::MarkerArray waypoint_area;
  visualization_msgs::MarkerArray waypoint_number_txt;
  waypoint_area.markers.resize(waypoint_csv_index_ + 1);
  waypoint_number_txt.markers.resize(waypoint_csv_index_ + 1);
  uint8_t vec_cnt_index(0);
  for (auto it_t = waypoint_csv_.begin(); it_t != waypoint_csv_.end(); ++it_t)
  {
    vec_cnt_index = 0;
    WaypointMarkerArraySet(waypoint_area, waypoint_number_txt,
                           distance(waypoint_csv_.begin(), it_t),
                           waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size());

    for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it)
    {
      switch (vec_cnt_index)
      {
        case 0:
          pose.position.x = stod(*it);
          if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
            waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x =
                stod(*it);
          waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x =
              stod(*it);
          vec_cnt_index++;
          continue;
        case 1:
          pose.position.y = stod(*it);
          if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
            waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y =
                stod(*it);
          waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y =
              stod(*it);
          vec_cnt_index++;
          continue;

          pose.position.z = 0.2;
          if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
            waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z = 0.1;
          waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z = 0.1;

        case 2:
          pose.orientation.z = stod(*it);
          vec_cnt_index++;
          continue;
        case 3:
          pose.orientation.w = stod(*it);
          vec_cnt_index++;
          break;
      }
    }
    pose_array.poses.push_back(pose);
  }
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";

  way_pose_array_.publish(pose_array);
  way_area_array_.publish(waypoint_area);
  way_number_txt_array_.publish(waypoint_number_txt);
}

void WaypointNav::WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area,
                                         visualization_msgs::MarkerArray& waypoint_number_txt,
                                         uint8_t index, uint8_t size)
{
  /*waypoint area_________________________________________________________*/
  waypoint_area.markers[index].header.frame_id = "map";
  waypoint_area.markers[index].header.stamp = ros::Time::now();
  waypoint_area.markers[index].id = index;
  waypoint_area.markers[index].type = visualization_msgs::Marker::CYLINDER;
  waypoint_area.markers[index].action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 cylinder;
  cylinder.x = waypoint_area_threshold_ * 2;
  cylinder.y = waypoint_area_threshold_ * 2;
  cylinder.z = 0.03;
  waypoint_area.markers[index].scale = cylinder;
  if (size == 4)
    waypoint_area.markers[index].color.a = 0.1f;
  else
    waypoint_area.markers[index].color.a = 0.000001f;
  waypoint_area.markers[index].color.b = 1.0f;
  waypoint_area.markers[index].color.g = 0.0f;
  waypoint_area.markers[index].color.r = 0.0f;
  waypoint_area.markers[index].pose.orientation.z = 0;
  waypoint_area.markers[index].pose.orientation.w = 1;
  /*______________________________________________________________________*/

  /*waypoint_number_txt________________________________________________________________*/
  waypoint_number_txt.markers[index].header.frame_id = "map";
  waypoint_number_txt.markers[index].header.stamp = ros::Time::now();
  waypoint_number_txt.markers[index].id = index;
  waypoint_number_txt.markers[index].text = to_string(index + 1);
  waypoint_number_txt.markers[index].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  waypoint_number_txt.markers[index].action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 text;
  text.z = waypoint_area_threshold_ / 2;
  waypoint_number_txt.markers[index].scale = text;
  waypoint_number_txt.markers[index].color.a = 1.0f;
  waypoint_number_txt.markers[index].color.b = 1.0f;
  waypoint_number_txt.markers[index].color.g = 1.0f;
  waypoint_number_txt.markers[index].color.r = 1.0f;
  /*___________________________________________________________________________________*/
}

void WaypointNav::WaypointInfoManagement()
{
  if (waypoint_csv_[waypoint_index_].size() >= 0 && waypoint_csv_[waypoint_index_].size() <= 4)
  {
    ModeFlagOff();
    NextWaypointMode_ = true;
  }
  else if (waypoint_csv_[waypoint_index_][4] == "Goal")
  {
    FinalGoalWaypointMode_ = true;
    if (waypoint_index_ == waypoint_csv_index_ && GoalReachedFlag_ && WaypointAreaCheck())
      FinalGoalFlag_ = true;
    if (waypoint_index_ == waypoint_csv_index_) ModeFlagOff();
    if (FinalGoalFlag_)
    {
      ROS_INFO("%s: Final Goal Reached", node_name_.c_str());
      ROS_INFO("%s: Please ' Ctl + c ' ", node_name_.c_str());
    }
  }
  else if (waypoint_csv_[waypoint_index_][4] == "GoalReStart")
  {
    ModeFlagOff();
    ReStartWaypointMode_ = true;
  }
  else if (waypoint_csv_[waypoint_index_][4] == "GoalReach")
  {
    ModeFlagOff();
    GoalReachedMode_ = true;
  }
}

bool WaypointNav::WaypointAreaCheck()
{
  if (NextWaypointMode_)
  {
    waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - robot_pose_[0], 2) +
                                pow(stod(waypoint_csv_[waypoint_index_][1]) - robot_pose_[1], 2));

    if (waypoint_area_check_ <= waypoint_area_threshold_)
    {
      ROS_INFO("%s: WayPoint Passing", node_name_.c_str());
      ROS_INFO("%s: Next Move Plan", node_name_.c_str());
      waypoint_index_++;
      std_msgs::Bool data;
      way_sound_.publish(data);
      return true;
    }
  }
  else if (!NextWaypointMode_)
  {
    waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - robot_pose_[0], 2) +
                                pow(stod(waypoint_csv_[waypoint_index_][1]) - robot_pose_[1], 2));

    if (waypoint_area_check_ <= waypoint_area_threshold_)
    {
      ROS_INFO("%s: Invade WayPoint Area ", node_name_.c_str());
      return true;
    }
  }
  return false;
}

bool WaypointNav::GoalReachCheck()
{
  if (GoalReachedFlag_)
  {
    ROS_INFO("%s: Goal Reached", node_name_.c_str());
    ROS_INFO("%s: Restart", node_name_.c_str());
    waypoint_index_++;
    return true;
  }
  return false;
}

void WaypointNav::WaypointSet(move_base_msgs::MoveBaseGoal& goal)
{
  goal.target_pose.pose.position.x = stod(waypoint_csv_[waypoint_index_][0]);
  goal.target_pose.pose.position.y = stod(waypoint_csv_[waypoint_index_][1]);
  goal.target_pose.pose.orientation.z = stod(waypoint_csv_[waypoint_index_][2]);
  goal.target_pose.pose.orientation.w = stod(waypoint_csv_[waypoint_index_][3]);
  goal.target_pose.header.stamp = ros::Time::now();

  ac_move_base_.sendGoal(goal);
}

void WaypointNav::ModeFlagOff()
{
  NextWaypointMode_ = false;
  FinalGoalWaypointMode_ = false;
  ReStartWaypointMode_ = false;
  GoalReachedMode_ = false;

  GoalReachedFlag_ = false;
  ReStartFlag_ = false;
}

void WaypointNav::ModeFlagDebug()
{
  cout << "___________________\n"
       << "NextWaypointMode : " << NextWaypointMode_ << "\n"
       << "FinalGoalWaypointMode : " << FinalGoalWaypointMode_ << "\n"
       << "ReStartWaypointMode : " << ReStartWaypointMode_ << "\n"
       << "GoalReachedMode : " << GoalReachedMode_ << "\n"
       << "GoalReachedFlag : " << GoalReachedFlag_ << "\n"

       << "~~~~~~~~~~~~~~~~~~~\n"
       << "WaypointIndex   : " << waypoint_index_ << "\n"
       << "___________________\n";
}

void WaypointNav::Run()
{
  goal_.target_pose.header.frame_id = "map";
  WaypointSet(goal_);

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if (NextWaypointMode_)
    {
      if (WaypointAreaCheck()) WaypointSet(goal_);
    }
    else if (FinalGoalWaypointMode_)
      WaypointSet(goal_);
    else if (ReStartWaypointMode_)
    {
      if (ReStartFlag_) WaypointSet(goal_);
    }
    else if (GoalReachedMode_)
    {
      if (WaypointAreaCheck() && GoalReachCheck()) WaypointSet(goal_);
    }
    // ModeFlagDebug();
    WaypointInfoManagement();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void WaypointNav::getRobotPose(const ros::TimerEvent&)
{
  geometry_msgs::PoseStamped global_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();

  std::string global_frame = "map";

  try
  {
    tf_.transform(robot_pose, global_pose, global_frame);
    robot_pose_.at(0) = global_pose.pose.position.x;
    robot_pose_.at(1) = global_pose.pose.position.y;
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
  }
}

void WaypointNav::GoalReachedCb(const actionlib_msgs::GoalStatusArray& status)
{
  if (!status.status_list.empty())
  {
    actionlib_msgs::GoalStatus goalStatus = status.status_list[0];

    if (goalStatus.status == 3 && GoalReachedFlag_ == false) GoalReachedFlag_ = true;
  }
}

void WaypointNav::GoalCommandCb(const std_msgs::String& msg)
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

void WaypointNav::WaypointStartCb(const std_msgs::String& msg)
{
  if (!MsgReceiveFlag_)
  {
    MsgReceiveFlag_ = true;
    Run();
  }
}

void WaypointNav::WaypointRestartCb(const std_msgs::String& msg)
{
  ReStartFlag_ = true;
  waypoint_index_++;
}

}  // namespace waypoint_nav
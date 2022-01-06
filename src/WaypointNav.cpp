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
WaypointNav::WaypointNav(ros::NodeHandle& nodeHandle, ros::NodeHandle& private_nodeHandle,
                         std::string node_name, std::string file_name, tf2_ros::Buffer& tf)
    : nh_(nodeHandle),
      pnh_(private_nodeHandle),
      tf_(tf),
      ac_move_base_("move_base", true),
      waypoint_server_loader_("raspicat_navigation", "raspicat_navigation::BaseWaypointServer"),
      waypoint_rviz_loader_("raspicat_navigation", "raspicat_navigation::BaseWaypointRviz"),
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
  readParam();
  initTimerCb();
  initPubSub();
  initActionClient();
  initClassLoader();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::readParam()
{
  pnh_.param("base_waypoint_server", waypoint_server_,
             std::string("raspicat_navigation/WaypointServer"));

  pnh_.param("base_waypoint_rviz", waypoint_rviz_, std::string("raspicat_navigation/WaypointRviz"));
}

void WaypointNav::initTimerCb()
{
  timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointNav::getRobotPose, this);
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
  while (!ac_move_base_.waitForServer(ros::Duration(100.0)))
  {
    ROS_ERROR("Waiting for the move_base action server to come up");
    exit(0);
  }
  ROS_INFO("MoveBase server comes up");
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
  catch (pluginlib::PluginlibException& ex)
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
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
}

void WaypointNav::Run()
{
  goal_.target_pose.header.frame_id = "map";
  setWaypoint(goal_);

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if (NextWaypointMode_)
    {
      if (checkWaypointArea()) setWaypoint(goal_);
    }
    else if (FinalGoalWaypointMode_)
      setWaypoint(goal_);
    else if (ReStartWaypointMode_)
    {
      if (ReStartFlag_) setWaypoint(goal_);
    }
    else if (GoalReachedMode_)
    {
      if (checkWaypointArea() && checkGoalReach()) setWaypoint(goal_);
    }
    // debugModeFlag();
    managementWaypointInfo();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool WaypointNav::checkWaypointArea()
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

void WaypointNav::setWaypoint(move_base_msgs::MoveBaseGoal& goal)
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

void WaypointNav::managementWaypointInfo()
{
  if (waypoint_csv_[waypoint_index_].size() >= 0 && waypoint_csv_[waypoint_index_].size() <= 4)
  {
    ModeFlagOff();
    NextWaypointMode_ = true;
  }
  else if (waypoint_csv_[waypoint_index_][4] == "Goal")
  {
    FinalGoalWaypointMode_ = true;
    if (waypoint_index_ == waypoint_csv_index_ && GoalReachedFlag_ && checkWaypointArea())
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
void WaypointNav::debugModeFlag()
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

bool WaypointNav::checkGoalReach()
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
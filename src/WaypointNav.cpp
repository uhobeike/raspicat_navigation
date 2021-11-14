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

#include "raspicat_navigation/WaypointNav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace ::std;

namespace waypoint_nav
{
WaypointNav::WaypointNav(ros::NodeHandle& nodeHandle, std::string node_name, std::string file_name)
    : nh_(nodeHandle),
      ac_move_base_("move_base", true),
      node_name_(node_name),
      csv_fname_(file_name),
      waypoint_csv_index_(1),
      waypoint_index_(0),
      waypoint_csv_(1, vector<string>(0)),
      amcl_pose_(4, 0),
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
  PubSub_Init();
  ActionClient_Init();

  WaypointCsvRead();
  WaypointRvizVisualization();
}

WaypointNav::~WaypointNav() {}

void WaypointNav::PubSub_Init()
{
  sub_amcl_pose_ = nh_.subscribe("amcl_pose", 1, &WaypointNav::AmclPoseCb, this);
  sub_movebase_goal_ = nh_.subscribe("move_base/status", 1, &WaypointNav::GoalReachedCb, this);
  sub_goal_command_ = nh_.subscribe("goal_command", 1, &WaypointNav::GoalCommandCb, this);

  way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
  way_area_array_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_area", 1, true);
  way_number_txt_array_ =
      nh_.advertise<visualization_msgs::MarkerArray>("waypoint_number_txt", 1, true);
}

void WaypointNav::ActionClient_Init()
{
  while (!ac_move_base_.waitForServer(ros::Duration(30.0)))
  {
    ROS_ERROR("Waiting for the move_base action server to come up");
    exit(0);
  }
  ROS_INFO("MoveBase server comes up");
}

void WaypointNav::WaypointCsvRead()
{
  ifstream f_r(csv_fname_.c_str(), std::ios::in);
  if (f_r.fail())
  {
    ROS_ERROR("std::ifstream could not open %s.", csv_fname_.c_str());
    exit(-1);
  }

  string line, word;
  while (getline(f_r, line))
  {
    istringstream stream(line);
    while (getline(stream, word, ','))
    {
      waypoint_csv_[waypoint_csv_index_ - 1].push_back(word);
    }
    waypoint_csv_.resize(++waypoint_csv_index_);
  }
  /*Index adjustment__________________________*/
  waypoint_csv_.resize(--waypoint_csv_index_);
  --waypoint_csv_index_;
  /*__________________________________________*/
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
    waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - amcl_pose_[0], 2) +
                                pow(stod(waypoint_csv_[waypoint_index_][1]) - amcl_pose_[1], 2));

    if (waypoint_area_check_ <= waypoint_area_threshold_)
    {
      ROS_INFO("%s: WayPoint Passing", node_name_.c_str());
      ROS_INFO("%s: Next Move Plan", node_name_.c_str());
      waypoint_index_++;
      return true;
    }
  }
  else if (!NextWaypointMode_)
  {
    waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - amcl_pose_[0], 2) +
                                pow(stod(waypoint_csv_[waypoint_index_][1]) - amcl_pose_[1], 2));

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
    ModeFlagDebug();
    WaypointInfoManagement();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void WaypointNav::AmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  amcl_pose_.at(0) = msg.pose.pose.position.x;
  amcl_pose_.at(1) = msg.pose.pose.position.y;
  amcl_pose_.at(2) = msg.pose.pose.orientation.z;
  amcl_pose_.at(3) = msg.pose.pose.orientation.w;
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

}  // namespace waypoint_nav
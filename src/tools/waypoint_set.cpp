/*
 *Copyright 2020, uhobeike.
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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::find;
using std::ofstream;
using std::stof;
using std::string;
using std::to_string;
using std::vector;

class waypoint_rviz
{
 private:
  ros::NodeHandle nh_;
  ros::Publisher way_pub_, marker_pub_;
  ros::Subscriber subscriber_set_, subscriber_control_;

  geometry_msgs::Point point_;
  geometry_msgs::PoseArray pose_array_;
  geometry_msgs::Pose pose_rviz_;
  std_msgs::ColorRGBA point_color_;
  visualization_msgs::Marker marker_;

  vector<string> posi_set_;
  vector<vector<string>> csv_array_;
  vector<uint16_t> waypoint_marker_id_;

  uint16_t waypoint_number_;
  uint16_t waypoint_index_;
  uint32_t shape_;
  bool marker_mode_;
  bool marker_flag_;

  string csv_path_;

  void waypoint_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
  void control_Callback(const std_msgs::StringConstPtr& command);

  void waypoint_csv_(vector<string>& posi_set, vector<vector<string>>& csv_array,
                     uint16_t& waypoint_number);
  void way_point_remove_(vector<vector<string>>& waypoint_remove,
                         geometry_msgs::PoseArray& pose_array, visualization_msgs::Marker& marker,
                         uint16_t& waypoint_number);
  void way_point_goal_set_(vector<vector<string>>& waypoint_goal, uint16_t& waypoint_number);
  void finish_and_file_write_waypoint_(vector<vector<string>>& waypoint_file_write,
                                       uint16_t& waypoint_number);
  void waypoint_marker_(visualization_msgs::Marker& marker, geometry_msgs::Point& point,
                        std_msgs::ColorRGBA& point_color,
                        vector<vector<string>>& waypoint_goal_corner, bool& marker_mode,
                        bool& marker_flag, uint16_t& waypoint_number);
  bool waypoint_marker_id_check(vector<uint16_t>& waypoint_marker_id, bool& marker_flag,
                                uint16_t& waypoint_number);

 public:
  waypoint_rviz(std::string file_name);
  ~waypoint_rviz(){};
};

waypoint_rviz::waypoint_rviz(std::string file_name) : nh_("")
{
  subscriber_set_ = nh_.subscribe("waypoint_set", 1, &waypoint_rviz::waypoint_Callback, this);
  subscriber_control_ =
      nh_.subscribe("waypoint_control", 1, &waypoint_rviz::control_Callback, this);

  way_pub_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  csv_path_ = file_name;

  posi_set_ = {"0", "0", "0", "0"};

  waypoint_number_ = 0;
  marker_mode_ = 0;
  marker_flag_ = 0;

  shape_ = visualization_msgs::Marker::POINTS;
}

void waypoint_rviz::waypoint_Callback(
    const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  pose_array_.header.stamp = ros::Time::now();
  pose_array_.header.frame_id = "map";

  pose_rviz_.position.x = pose->pose.position.x;
  pose_rviz_.position.y = pose->pose.position.y;
  pose_rviz_.position.z = 0.2;
  pose_rviz_.orientation.z = pose->pose.orientation.z;
  pose_rviz_.orientation.w = pose->pose.orientation.w;

  pose_array_.poses.push_back(pose_rviz_);

  way_pub_.publish(pose_array_);

  posi_set_.at(0) = to_string(pose->pose.position.x);
  posi_set_.at(1) = to_string(pose->pose.position.y);
  posi_set_.at(2) = to_string(pose->pose.orientation.z);
  posi_set_.at(3) = to_string(pose->pose.orientation.w);

  waypoint_csv_(posi_set_, csv_array_, waypoint_number_);
}

void waypoint_rviz::control_Callback(const std_msgs::StringConstPtr& command)
{
  if (command->data == "remove")
  {
    way_point_remove_(csv_array_, pose_array_, marker_, waypoint_number_);
  }

  else if (command->data == "goal")
  {
    way_point_goal_set_(csv_array_, waypoint_number_);
  }

  else if (command->data == "finish")
  {
    finish_and_file_write_waypoint_(csv_array_, waypoint_number_);
  }
}

void waypoint_rviz::waypoint_csv_(vector<string>& posi_set, vector<vector<string>>& csv_array,
                                  uint16_t& waypoint_number)
{
  csv_array.push_back(posi_set);
  waypoint_number++;
}

void waypoint_rviz::way_point_remove_(vector<vector<string>>& waypoint_remove,
                                      geometry_msgs::PoseArray& pose_array,
                                      visualization_msgs::Marker& marker, uint16_t& waypoint_number)
{
  ROS_INFO("way_point_remove");

  if (0 < waypoint_number)
  {
    waypoint_number--;
    waypoint_remove.resize(waypoint_number);
    pose_array.poses.resize(waypoint_number);

    way_pub_.publish(pose_array_);
  }

  if (waypoint_marker_id_check(waypoint_marker_id_, marker_flag_, waypoint_number))
  {
    marker_.points.pop_back();
    marker.colors.pop_back();

    marker_pub_.publish(marker);
  }
}

void waypoint_rviz::way_point_goal_set_(vector<vector<string>>& waypoint_goal,
                                        uint16_t& waypoint_number)
{
  uint16_t array_number = waypoint_number - 1;
  waypoint_goal[array_number].push_back("Goal");

  marker_mode_ = 0;
  waypoint_marker_(marker_, point_, point_color_, waypoint_goal, marker_mode_, marker_flag_,
                   waypoint_number);
}

void waypoint_rviz::finish_and_file_write_waypoint_(vector<vector<string>>& waypoint_file_write,
                                                    uint16_t& waypoint_number)
{
  ROS_INFO("finish_and_file_write_waypoint q(^_^)p");
  ROS_INFO("%s", csv_path_.c_str());

  ofstream f_w(csv_path_ + "waypoint.csv", std::ios::out);

  for (auto it_t = waypoint_file_write.begin(); it_t != waypoint_file_write.end(); ++it_t)
  {
    for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it)
    {
      f_w << *it << ",";
      cout << *it << endl;
    }

    f_w << endl;
  }

  f_w.close();

  ros::shutdown();
}

void waypoint_rviz::waypoint_marker_(visualization_msgs::Marker& marker,
                                     geometry_msgs::Point& point, std_msgs::ColorRGBA& point_color,
                                     vector<vector<string>>& waypoint_goal_corner,
                                     bool& marker_mode, bool& marker_flag,
                                     uint16_t& waypoint_number)
{
  uint16_t array_number = waypoint_number - 1;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.id = 0;
  marker.type = shape_;

  marker.action = visualization_msgs::Marker::ADD;

  point.x = stof(waypoint_goal_corner[array_number][0]);
  point.y = stof(waypoint_goal_corner[array_number][1]);
  point.z = 0.2;

  marker.points.push_back(point);

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;

  if (!marker_mode)
  {
    point_color.r = 1.0f;
    point_color.g = 0.0f;
    point_color.b = 0.0f;
    point_color.a = 1.0;
  }

  else if (marker_mode)
  {
    point_color.r = 0.0f;
    point_color.g = 0.0f;
    point_color.b = 1.0f;
    point_color.a = 1.0;
  }

  marker.colors.push_back(point_color);

  waypoint_marker_id_.push_back(array_number);

  marker_pub_.publish(marker);

  marker_flag = 1;
}

bool waypoint_rviz::waypoint_marker_id_check(vector<uint16_t>& waypoint_marker_id,
                                             bool& marker_flag, uint16_t& waypoint_number)
{
  uint8_t flag = 0;
  if (marker_flag)
  {
    auto result = find(waypoint_marker_id.begin(), waypoint_marker_id.end(), waypoint_number);

    if (*result == waypoint_number)
    {
      waypoint_marker_id.pop_back();
      flag = 1;
    }
  }

  return flag;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_rviz_set");

  waypoint_rviz node(argv[1]);

  ros::spin();

  return 0;
}

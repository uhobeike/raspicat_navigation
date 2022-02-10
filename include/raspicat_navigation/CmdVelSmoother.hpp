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

#ifndef CMD_VEL_SMOOTHER_HPP_
#define CMD_VEL_SMOOTHER_HPP_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <vector>

#include "raspicat_navigation/WaypointNavHelperPlugin.hpp"

namespace raspicat_navigation
{
class CmdVelSmoother : public raspicat_navigation::WaypointNavHelperPlugin
{
  ros::NodeHandle nh_, pnh_;
  ros::Publisher cmd_vel_publisher_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Timer SmoothCmdVelTimer_;
  ros::Duration SmoothCmdVelDuration_;

  ros::Time latest_cmdvel_time_, oldest_cmdvel_time_;
  std::vector<geometry_msgs::Twist> cmd_vel_que_;
  geometry_msgs::Twist publish_cmd_vel_;
  double acc_limit_x_;
  double acc_limit_z_;
  double dec_x_;
  double dec_z_;
  bool publish_flag_;

 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::CmdVelSmoother initialize"); }
  void run()
  {
    ROS_INFO("raspicat_navigation::CmdVelSmoother run");
    readParameters();
    initPubSub();
  }

  void readParameters();
  void initPubSub();
  void calculateDeceleration(double pass_sec);
  void publishSmoothVelcity();
};

}  // namespace raspicat_navigation
#endif  // CMD_VEL_SMOOTHER_HPP_
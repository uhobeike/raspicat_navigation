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

#ifndef SLOPE_OBSTACLE_AVOIDANCE_HPP_
#define SLOPE_OBSTACLE_AVOIDANCE_HPP_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>

#include "raspicat_waypoint_navigation/WaypointNavHelperPlugin.hpp"

namespace raspicat_navigation
{
class SlopeObstacleAvoidance : public raspicat_navigation::WaypointNavHelperPlugin
{
  ros::NodeHandle nh_, pnh_;
  ros::Publisher filter_scan_publisher_;
  ros::Subscriber scan_subscriber_;
  ros::ServiceServer srv_scan_filter_on_, srv_scan_filter_off_;

  ros::Timer checkScanPramTimer_;
  ros::Time latest_scan_time_;
  ros::Duration checkScanParamDuration_;
  sensor_msgs::LaserScan out_scan_;
  double scan_receive_tolerance_;
  float scan_range_limit_;
  bool in_scan_, scan_filter_;

 public:
  void initialize(std::string name)
  {
    ROS_INFO("raspicat_navigation::SlopeObstacleAvoidance initialize");
    scan_filter_ = false;
  }
  void run()
  {
    ROS_INFO("raspicat_navigation::SlopeObstacleAvoidance run");
    readParameters();
    initPubSub();
    initServiceServer();
    initTimerCb();
  }

  void readParameters();
  void initPubSub();
  void initServiceServer();
  void initTimerCb();

  void filterScan(sensor_msgs::LaserScan &out_scan);
};

}  // namespace raspicat_navigation
#endif  // SLOPE_OBSTACLE_AVOIDANCE
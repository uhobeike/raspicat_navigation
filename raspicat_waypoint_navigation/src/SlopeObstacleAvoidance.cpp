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

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_waypoint_navigation/SlopeObstacleAvoidance.hpp"

namespace raspicat_navigation
{
void SlopeObstacleAvoidance::readParameters()
{
  float obstacle_range;
  while (!pnh_.getParam("/move_base/local_costmap/obstacles_layer/obstacle_range", obstacle_range))
  {
    ROS_ERROR("Cannot read /move_base/local_costmap/obstacles_layer/obstacle_range parameters.");
    ros::Duration duration(0.1);
    duration.sleep();
  }

  pnh_.param("scan_recive_tolerance", scan_receive_tolerance_, 1.0);
  pnh_.param("obstacle_range_limit", scan_range_limit_, static_cast<float>(1.0));  // obstacle_range

  double checkRate;
  pnh_.param("check_scan_param_rate", checkRate, 10.0);
  if (checkRate == 0.0)
  {
    checkScanParamDuration_.fromSec(0.0);
    ROS_WARN("The rate for checking the value of scan_receive is 0.");
  }
  else
  {
    checkScanParamDuration_.fromSec(1.0 / checkRate);
  }
  ROS_ASSERT(!checkScanParamDuration_.isZero());
}

void SlopeObstacleAvoidance::initPubSub()
{
  filter_scan_publisher_ = pnh_.advertise<sensor_msgs::LaserScan>("/filter_scan", 1);

  scan_subscriber_ = pnh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, [&](const auto &in_scan) {
    out_scan_ = *in_scan;
    in_scan_ = true;
    latest_scan_time_ = ros::Time::now();
    if (scan_filter_) filterScan(out_scan_);
    filter_scan_publisher_.publish(out_scan_);
  });
}

void SlopeObstacleAvoidance::initServiceServer()
{
  srv_scan_filter_on_ = nh_.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      "slope_obstacle_avoidance_on", [&](auto &req, auto &res) {
        ROS_INFO("Called service slope_obstacle_avoidance_on.");
        scan_filter_ = true;
        res.message = "Slope Obstacle Avoidance On";
        res.success = true;
        return true;
      });

  srv_scan_filter_off_ = nh_.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      "slope_obstacle_avoidance_off", [&](auto &req, auto &res) {
        ROS_INFO("Called service slope_obstacle_avoidance_off.");
        scan_filter_ = false;
        res.message = "Slope Obstacle Avoidance Off";
        res.success = true;
        return true;
      });
}

void SlopeObstacleAvoidance::initTimerCb()
{
  checkScanPramTimer_ = pnh_.createTimer(checkScanParamDuration_, [&](auto &) {
    if (in_scan_ and
        ros::Time::now().toSec() - latest_scan_time_.toSec() >= scan_receive_tolerance_)
    {
      ROS_WARN("Not receiving a scan within %f seconds. It actually takes %f seconds.",
               scan_receive_tolerance_, (ros::Time::now().toSec() - latest_scan_time_.toSec()));
    }

    pnh_.getParam("obstacle_range_limit", scan_range_limit_);
  });
}

void SlopeObstacleAvoidance::filterScan(sensor_msgs::LaserScan &out_scan)
{
  for (auto &scan_range : out_scan.ranges)
  {
    if (scan_range > scan_range_limit_) scan_range = 0;
  }
}

}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::SlopeObstacleAvoidance,
                       raspicat_navigation::WaypointNavHelperPlugin)
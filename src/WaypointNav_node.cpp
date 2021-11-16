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
#include <tf2_ros/transform_listener.h>

#include "raspicat_navigation/WaypointNav.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "WaypointNav");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  waypoint_nav::WaypointNav wv(nh, ros::this_node::getName(), argv[1], buffer);
  ROS_INFO("%s: Please ' rostopic pub  -1 /goal_command std_msgs/String go ' ",
           ros::this_node::getName().c_str());

  ros::AsyncSpinner spinner(pnh.param("num_callback_threads", 1));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
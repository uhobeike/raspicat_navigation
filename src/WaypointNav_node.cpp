/*
*Copyright 2021, uhobeike.
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

#include "raspicat_navigation/WaypointNav.hpp"

using namespace::std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "WaypointNav");
	ros::NodeHandle nh;

	waypoint_nav::WaypointNav wv(nh, ros::this_node::getName(), argv[1]);
	ROS_INFO("%s: Please ' rostopic pub  -1 /goal_command std_msgs/String go ' ",
			 ros::this_node::getName().c_str());
	ros::spin();
	return 0;
}
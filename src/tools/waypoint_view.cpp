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

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <string>
#include <fstream> 
#include <sstream>

using std::vector;
using std::string;
using std::stoi;
using std::stod;
using std::ifstream;
using std::istringstream;
using std::cout;
using std::endl;

string vec_num = "0";

void waypoint_pose_array(vector<vector<string>>& waypoint_read, geometry_msgs::PoseArray& pose_array, geometry_msgs::Pose pose)
{
    uint16_t vec_cnt_out = 0, vec_cnt_in = 0;
    for (auto it_t = waypoint_read.begin(); it_t != waypoint_read.end(); ++it_t) 
    {
        vec_cnt_in = 0;
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it) 
        {
            vec_cnt_in++;
            if(vec_cnt_in == 5) break;

            if(vec_cnt_in == 1) pose.position.x = stod(*it);
            if(vec_cnt_in == 2) pose.position.y = stod(*it);
            
            pose.position.z = 0.2;
            
            if(vec_cnt_in == 3) pose.orientation.z = stod(*it);
            if(vec_cnt_in == 4) pose.orientation.w = stod(*it);

        }

        pose_array.poses.push_back(pose);
    }
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "raspicat_waypoint_view");
    ros::NodeHandle nh;
    ros::Publisher  pub_pose_way;

    pub_pose_way = nh.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);

    string csv_file = argv[1];

    ifstream f_r(csv_file + "waypoint.csv",std::ios::in);

    vector<vector<string>> waypoint_read;
    string line,field;
    int vec_num_int = 1;

    waypoint_read.emplace_back();

    while (getline(f_r, line)) 
    {
        istringstream stream(line);
        while (getline(stream, field, ',') )
        {
            waypoint_read[vec_num_int-1].push_back(field);
        }

        waypoint_read.resize(++vec_num_int);
    }
    waypoint_read.resize(--vec_num_int);
    
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    pose_array.header.stamp = ros::Time::now(); 
    pose_array.header.frame_id = "map"; 
    
    waypoint_pose_array(waypoint_read, pose_array, pose);

    pub_pose_way.publish(pose_array);

    ROS_INFO("waypoint published");

    ros::Rate loop_rate(1);//10Hz

    while(ros::ok())
    {  
        //re_publish code I'll do it later
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/String.h>
#include <iterator>
#include <string>
#include <algorithm>

using std::cin;
using std::iswdigit;
using std::string;
using std::stoi;

int sub_flag;
int index_cnt;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

string getKey(string& key)
{
    ROS_INFO("go Comand Only ,but 'q' is shutdown");

    while (ros::ok())
    {
        cin >> key;
        
        if(key == "go" || key == "q") break;

        else ROS_INFO("go Comand Only ,but 'q' is shutdown");
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "goal_key_send");
    ros::NodeHandle nh;
    ros::Publisher pub;

    pub = nh.advertise<std_msgs::String>("goal_key", 1, true);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Rate loop_rate(10);//10Hz

    while (ros::ok())
    {
        std_msgs::String msg_key;
        
        getKey(msg_key.data);

        if(msg_key.data == "go") pub.publish(msg_key);

        else if (msg_key.data == "q") 
        {
            pub.publish(msg_key);
            ros::shutdown();
        }
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

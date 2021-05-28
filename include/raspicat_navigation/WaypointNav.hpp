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

#ifndef WAYPOINT_NAV_
#define WAYPOINT_NAV_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

using namespace::std;

namespace waypoint_nav {

class WaypointNav
{
public:
    WaypointNav(ros::NodeHandle& nodeHandle, std::string name, std::string file_name);
    virtual ~WaypointNav();

    void AmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void GoalReachedCb(const actionlib_msgs::GoalStatusArray& status);
    void GoalCommandCb(const std_msgs::String& msg);

    void ActionClient_Init();
    void PubSub_Init();

    void WaypointCsvRead();
    void WaypointRvizVisualization();
    void WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area, 
                                visualization_msgs::MarkerArray& waypoint_number_txt,
                                uint8_t index, uint8_t siz);
    void WaypointInfoManagement();
    bool WaypointAreaCheck();
    bool GoalReachCheck();
    bool ObjectDetectCheck();

    void WaypointSet(move_base_msgs::MoveBaseGoal& next);

    void ModeFlagOff();
    void Run();

    void ModeFlagDebug();

private:
    ros::NodeHandle& nh_;

    ros::Subscriber sub_amcl_pose_, sub_movebase_goal_, sub_goal_command_;
    ros::Publisher ini_pose_, way_pose_array_, way_area_array_, way_number_txt_array_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

    string node_name_;

    string csv_fname_;
    int waypoint_csv_index_;
    int waypoint_index_;
    vector<vector<string>> waypoint_csv_;
    vector<double> amcl_pose_;  

    float waypoint_area_threshold_;
    float waypoint_area_check_;

    move_base_msgs::MoveBaseGoal goal_;

    bool NextWaypointMode_;
    bool FinalGoalWaypointMode_;
    bool ReStartWaypointMode_;
    bool GoalReachedMode_;

    bool FinalGoalFlag_;
    bool ReStartFlag_; 
    bool MsgReceiveFlag_;
    bool GoalReachedFlag_;
};

} /* namespace */
#endif
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include <fstream> 
#include <sstream>
#include <cmath>

using std::vector;
using std::string;
using std::stoi;
using std::stod;
using std::ifstream;
using std::istringstream;
using std::pow;
using std::cout;
using std::endl;

bool start_flag;
bool lock_flag;
bool goal_reached_flag;
bool goal_restart_flag;
bool final_goal_flag;
string vec_num = "0";
vector<double> posi_set = 
{
    0,0,0,0
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goal_key_Callback(const std_msgs::StringConstPtr& msg)
{
    if(msg->data == "q")
    {
        ROS_INFO("Shutdown now ('o')/ bye bye~~~");
        ros::shutdown();
    }

    ROS_INFO("Received a control command");
    
    if(start_flag) goal_restart_flag = 1;
    
    start_flag = 1;
}

void posi_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    posi_set.at(0) = msg->pose.pose.position.x;
    posi_set.at(1) = msg->pose.pose.position.y;
    posi_set.at(2) = msg->pose.pose.orientation.z;
    posi_set.at(3) = msg->pose.pose.orientation.w;
}  

void goal_reached_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    if (!status->status_list.empty() )
    {
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];

        if(goalStatus.status == 3 && goal_reached_flag == 0 && lock_flag == 0) goal_reached_flag = 1;
        
        if(goalStatus.status != 0 && goalStatus.status != 3) lock_flag = 0;
    }
}  

void goal_check(vector<vector<string>>& waypoint, int& point_number, int& vec_size, int& next_point_flag, int& goal_point_flag)
{   
    if(waypoint[point_number].size() >= 0 && waypoint[point_number].size() <= 4)
    {
        next_point_flag = 1;
    }
    else if(waypoint[point_number].size() > 4)
    {
        goal_point_flag = 1;
    }

    if(point_number == (vec_size - 2) && goal_reached_flag) final_goal_flag = 1;

    if(goal_reached_flag)
    {
        ROS_INFO("Goal reached");
        ROS_INFO("go Comand Only ,but 'q' is shutdown");
        goal_reached_flag = 0;
        lock_flag = 1;
    }

    if(goal_restart_flag)
    {
        ROS_INFO("RESTART");

        point_number++;

        goal_restart_flag = 0;
    }

    if(final_goal_flag)
    {
        ROS_INFO("Final goal reached");
        ROS_INFO("please send q command");
    }
}

void waypoint_nearly_check(vector<vector<string>>& waypoint, vector<double>& estimated_pos, int& point_number, int& goal_point_flag, double& area_threshold)
{
    double nealy_check_area = 0;
    double x_way = 0,y_way = 0,x_posi = 0,y_posi = 0;
    x_way = stod(waypoint[point_number][0]);
    y_way = stod(waypoint[point_number][1]);
    x_posi = estimated_pos[0];
    y_posi = estimated_pos[1];
    nealy_check_area = sqrt(pow( x_way - x_posi, 2) + pow( y_way - y_posi, 2) );
    
    if(nealy_check_area <= area_threshold)
    {
        ROS_INFO("WAY_POINT PASSING");
        ROS_INFO("NEXT MOVE PLAN");

        point_number++;
    }
}

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
    ros::init(argc, argv, "raspicat_waypoint_navigation");
    ros::NodeHandle nh;
    ros::Publisher pub_pose_ini, pub_pose_way;
    ros::Time tmp_time = ros::Time::now();
    nh.setParam("waypoint_area_threshold", 0.8);

    ros::Subscriber sub_key = nh.subscribe("goal_key", 1,  goal_key_Callback);
    ros::Subscriber sub_pos = nh.subscribe("amcl_pose", 1,  posi_Callback);
    ros::Subscriber sub_goal = nh.subscribe("move_base/status", 1,  goal_reached_Callback);

    pub_pose_ini = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    pub_pose_way = nh.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
/*
    geometry_msgs::PoseWithCovarianceStamped initPose;
    initPose.header.stamp = tmp_time; 
    initPose.header.frame_id = "map"; 
    initPose.pose.pose.position.x = 0;
    initPose.pose.pose.position.y = 0;
    initPose.pose.pose.position.z = 0;
    initPose.pose.pose.orientation.w = 1;
*/
    vector<vector<string>> waypoint_read = 
    {
        {"7.680957","-0.000170","0.002367","0.999997"},
        {"19.090723","0.414794","0.365262","0.930905"},
        {"20.049221","2.535768","0.712786","0.701381"},
        {"20.109728","8.462751","0.860792","0.508957"},
        {"18.021887","10.236370","-0.999981","0.006091"},
        {"7.942166","9.938683","-0.999962","0.008733"},
        {"-6.659242","9.561300","0.999995","0.003110"},
        {"-13.046364","9.733968","-0.993011","0.118023"},
        {"-14.348382","8.901632","-0.706335","0.707878"},
        {"-14.442686","6.326262","-0.687793","0.725907"},
        {"-14.282948","2.820924","-0.693112","0.720830"},
        {"-14.296299","0.524044","-0.364502","0.931203"},
        {"-12.451824","-0.086312","-0.001220","0.999999"},
        {"-0.320502","0.118533","0.003982","0.999992","goal"}
    };

    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    pose_array.header.stamp = ros::Time::now(); 
    pose_array.header.frame_id = "map"; 
    
    waypoint_pose_array(waypoint_read, pose_array, pose);

    pub_pose_way.publish(pose_array);

    //pub_pose_ini.publish(initPose);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";                                                                     
    goal.target_pose.header.stamp = ros::Time::now();

    ros::Rate loop_rate(5);//10Hz

    int vec_size = waypoint_read.size();
    int point_number=0;
    int next_point_flag = 0;
    int goal_point_flag = 0;
    double area_threshold = 0.2;
    while(ros::ok())
    {  
        nh.getParam("waypoint_area_threshold", area_threshold);

        if(start_flag)
        {
            goal_check(waypoint_read,  point_number, vec_size, next_point_flag, goal_point_flag);

            if(next_point_flag)
            {        
                goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
                goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
                goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
                goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

                ac.sendGoal(goal);

                waypoint_nearly_check(waypoint_read, posi_set, point_number, goal_point_flag, area_threshold);
                next_point_flag = 0; 
            }

            else if (goal_point_flag)
            {
                goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
                goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
                goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
                goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

                ac.sendGoal(goal);

                goal_point_flag = 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

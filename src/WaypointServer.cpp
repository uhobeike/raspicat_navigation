#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <sstream>

#include "raspicat_navigation/BaseWaypointServer.hpp"

// using namespace ::std;

namespace raspicat_navigation
{
class WaypointServer : public raspicat_navigation::BaseWaypointServer
{
 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::WaypointServer initialize"); }
  void run() { ROS_INFO("raspicat_navigation::WaypointServer run"); }
  void WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
                       int &waypoint_csv_index_)
  {
    ifstream f_r(csv_fname_.c_str(), std::ios::in);
    if (f_r.fail())
    {
      ROS_ERROR("std::ifstream could not open %s.", csv_fname_.c_str());
      exit(-1);
    }

    string line, word;
    while (getline(f_r, line))
    {
      istringstream stream(line);
      while (getline(stream, word, ','))
      {
        waypoint_csv_[waypoint_csv_index_ - 1].push_back(word);
      }
      waypoint_csv_.resize(++waypoint_csv_index_);
    }
    /*Index adjustment__________________________*/
    waypoint_csv_.resize(--waypoint_csv_index_);
    --waypoint_csv_index_;
    /*__________________________________________*/
  }

  void setWaypoint(move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_,
                   int &waypoint_index_,
                   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_)
  {
    goal.target_pose.pose.position.x = stod(waypoint_csv_[waypoint_index_][0]);
    goal.target_pose.pose.position.y = stod(waypoint_csv_[waypoint_index_][1]);
    goal.target_pose.pose.orientation.z = stod(waypoint_csv_[waypoint_index_][2]);
    goal.target_pose.pose.orientation.w = stod(waypoint_csv_[waypoint_index_][3]);
    goal.target_pose.header.stamp = ros::Time::now();

    ac_move_base_.sendGoal(goal);
  }

  bool checkWaypointArea(bool &NextWaypointMode_, float &waypoint_area_check_,
                         vector<vector<string>> &waypoint_csv_, int &waypoint_index_,
                         vector<double> &robot_pose_, float &waypoint_area_threshold_,
                         string &node_name_, ros::Publisher &way_sound_)
  {
    if (NextWaypointMode_)
    {
      waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - robot_pose_[0], 2) +
                                  pow(stod(waypoint_csv_[waypoint_index_][1]) - robot_pose_[1], 2));

      if (waypoint_area_check_ <= waypoint_area_threshold_)
      {
        ROS_INFO("%s: WayPoint Passing", node_name_.c_str());
        ROS_INFO("%s: Next Move Plan", node_name_.c_str());
        waypoint_index_++;
        std_msgs::Bool data;
        way_sound_.publish(data);
        return true;
      }
    }
    else if (!NextWaypointMode_)
    {
      waypoint_area_check_ = sqrt(pow(stod(waypoint_csv_[waypoint_index_][0]) - robot_pose_[0], 2) +
                                  pow(stod(waypoint_csv_[waypoint_index_][1]) - robot_pose_[1], 2));

      if (waypoint_area_check_ <= waypoint_area_threshold_)
      {
        ROS_INFO("%s: Invade WayPoint Area ", node_name_.c_str());
        return true;
      }
    }
    return false;
  }

  void getRobotPose(tf2_ros::Buffer &tf_, vector<double> &robot_pose_)
  {
    geometry_msgs::PoseStamped global_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();

    std::string global_frame = "map";

    try
    {
      tf_.transform(robot_pose, global_pose, global_frame);
      robot_pose_.at(0) = global_pose.pose.position.x;
      robot_pose_.at(1) = global_pose.pose.position.y;
    }
    catch (tf2::LookupException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n",
                         ex.what());
    }
    catch (tf2::ConnectivityException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    }
    catch (tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    }
  }

  bool checkGoalReach(bool &GoalReachedFlag_, string &node_name_, int &waypoint_index_)
  {
    if (GoalReachedFlag_)
    {
      ROS_INFO("%s: Goal Reached", node_name_.c_str());
      ROS_INFO("%s: Restart", node_name_.c_str());
      waypoint_index_++;
      return true;
    }
    return false;
  }

  void ModeFlagOff(bool &NextWaypointMode_, bool &FinalGoalWaypointMode_,
                   bool &ReStartWaypointMode_, bool &GoalReachedMode_, bool &ReStartFlag_,
                   bool &GoalReachedFlag_)
  {
    NextWaypointMode_ = false;
    FinalGoalWaypointMode_ = false;
    ReStartWaypointMode_ = false;
    GoalReachedMode_ = false;

    GoalReachedFlag_ = false;
    ReStartFlag_ = false;
  }

  void managementWaypointInfo(vector<vector<string>> &waypoint_csv_, int &waypoint_csv_index_,
                              int &waypoint_index_, string &node_name_, bool &NextWaypointMode_,
                              bool &FinalGoalWaypointMode_, bool &ReStartWaypointMode_,
                              bool &GoalReachedMode_, bool &ReStartFlag_, bool &GoalReachedFlag_,
                              bool &FinalGoalFlag_, float &waypoint_area_check_,
                              vector<double> &robot_pose_, float &waypoint_area_threshold_,
                              ros::Publisher &way_sound_)
  {
    if (waypoint_csv_[waypoint_index_].size() >= 0 && waypoint_csv_[waypoint_index_].size() <= 4)
    {
      ModeFlagOff(NextWaypointMode_, FinalGoalWaypointMode_, ReStartWaypointMode_, GoalReachedMode_,
                  ReStartFlag_, GoalReachedFlag_);
      NextWaypointMode_ = true;
    }
    else if (waypoint_csv_[waypoint_index_][4] == "Goal")
    {
      FinalGoalWaypointMode_ = true;
      if (waypoint_index_ == waypoint_csv_index_ && GoalReachedFlag_ &&
          checkWaypointArea(NextWaypointMode_, waypoint_area_check_, waypoint_csv_, waypoint_index_,
                            robot_pose_, waypoint_area_threshold_, node_name_, way_sound_))
        FinalGoalFlag_ = true;
      if (waypoint_index_ == waypoint_csv_index_)
        ModeFlagOff(NextWaypointMode_, FinalGoalWaypointMode_, ReStartWaypointMode_,
                    GoalReachedMode_, ReStartFlag_, GoalReachedFlag_);
      if (FinalGoalFlag_)
      {
        ROS_INFO("%s: Final Goal Reached", node_name_.c_str());
        ROS_INFO("%s: Please ' Ctl + c ' ", node_name_.c_str());
      }
    }
    else if (waypoint_csv_[waypoint_index_][4] == "GoalReStart")
    {
      ModeFlagOff(NextWaypointMode_, FinalGoalWaypointMode_, ReStartWaypointMode_, GoalReachedMode_,
                  ReStartFlag_, GoalReachedFlag_);
      ReStartWaypointMode_ = true;
    }
    else if (waypoint_csv_[waypoint_index_][4] == "GoalReach")
    {
      ModeFlagOff(NextWaypointMode_, FinalGoalWaypointMode_, ReStartWaypointMode_, GoalReachedMode_,
                  ReStartFlag_, GoalReachedFlag_);
      GoalReachedMode_ = true;
    }
  }

  void debugModeFlag(bool &NextWaypointMode_, bool &FinalGoalWaypointMode_,
                     bool &ReStartWaypointMode_, bool &GoalReachedMode_, bool &GoalReachedFlag_,
                     int &waypoint_index_)
  {
    cout << "___________________\n"
         << "NextWaypointMode : " << NextWaypointMode_ << "\n"
         << "FinalGoalWaypointMode : " << FinalGoalWaypointMode_ << "\n"
         << "ReStartWaypointMode : " << ReStartWaypointMode_ << "\n"
         << "GoalReachedMode : " << GoalReachedMode_ << "\n"
         << "GoalReachedFlag : " << GoalReachedFlag_ << "\n"

         << "~~~~~~~~~~~~~~~~~~~\n"
         << "WaypointIndex   : " << waypoint_index_ << "\n"
         << "___________________\n";
  }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
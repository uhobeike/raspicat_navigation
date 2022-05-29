#include "raspicat_navigation/WaypointServer.hpp"

namespace raspicat_navigation
{
void WaypointServer::initialize(std::string name)
{
  ROS_INFO("raspicat_navigation::WaypointServer initialize");
}
void WaypointServer::run() { ROS_INFO("raspicat_navigation::WaypointServer run"); }
void WaypointServer::WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
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

void WaypointServer::checkWaypointYmal(ros::NodeHandle &pnh)
{
  XmlRpc::XmlRpcValue waypoint_yaml_;
  pnh.getParam("waypoints", waypoint_yaml_);
  for (auto i = 0; i < waypoint_yaml_.size(); ++i)
  {
    cout << waypoint_yaml_[i]["position"]["x"] << "\n";
    cout << waypoint_yaml_[i]["position"]["y"] << "\n";
    cout << waypoint_yaml_[i]["euler_angle"]["z"] << "\n";

    for (auto j = 0; j < waypoint_yaml_[i]["properties"].size(); ++j)
    {
      cout << (waypoint_yaml_[i]["properties"][j]["function"]) << "\n";

      if (waypoint_yaml_[i]["properties"][j]["function"] == "attention_speak")
      {
        cout << waypoint_yaml_[i]["properties"][j]["speak_interval"] << "\n";
      }

      else if (waypoint_yaml_[i]["properties"][j]["function"] == "slop")
      {
        cout << waypoint_yaml_[i]["properties"][j]["scan_range_limit"] << "\n";
      }

      else if (waypoint_yaml_[i]["properties"][j]["function"] == "step")
      {
        cout << waypoint_yaml_[i]["properties"][j]["approach_run"] << "\n";
        cout << waypoint_yaml_[i]["properties"][j]["power"] << "\n";
      }

      else if (waypoint_yaml_[i]["properties"][j]["function"] == "variable_speed")
      {
        cout << waypoint_yaml_[i]["properties"][j]["linear"] << "\n";
      }

      else if (waypoint_yaml_[i]["properties"][j]["function"] == "variable_waypoint_radius")
      {
        cout << waypoint_yaml_[i]["properties"][j]["radius"] << "\n";
      }
    }
  }
}

void WaypointServer::setWaypoint(
    move_base_msgs::MoveBaseGoal &goal, vector<vector<string>> &waypoint_csv_, int &waypoint_index_,
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base_)
{
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = stod(waypoint_csv_[waypoint_index_][0]);
  goal.target_pose.pose.position.y = stod(waypoint_csv_[waypoint_index_][1]);
  goal.target_pose.pose.orientation.z = stod(waypoint_csv_[waypoint_index_][2]);
  goal.target_pose.pose.orientation.w = stod(waypoint_csv_[waypoint_index_][3]);
  goal.target_pose.header.stamp = ros::Time::now();

  ac_move_base_.sendGoal(goal);
}

bool WaypointServer::checkWaypointArea(
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
    vector<vector<string>> &waypoint_csv, int &waypoint_index, ros::Publisher &way_passed)
{
  if (WaypointNavStatus.next_waypoint_mode || WaypointNavStatus.slope_obstacle_avoidance_mode)
  {
    WaypointNavStatus.waypoint_current_distance = sqrt(
        pow(stod(waypoint_csv[waypoint_index][0]) - WaypointNavStatus.robot_pose.position.x, 2) +
        pow(stod(waypoint_csv[waypoint_index][1]) - WaypointNavStatus.robot_pose.position.y, 2));

    if (WaypointNavStatus.waypoint_current_distance <= WaypointNavStatus.waypoint_radius_threshold)
    {
      ROS_INFO("WayPoint Passing");
      ROS_INFO("Next Move Plan");
      waypoint_index++;
      std_msgs::Bool data;
      way_passed.publish(data);
      return true;
    }
  }
  else if (!WaypointNavStatus.next_waypoint_mode)
  {
    WaypointNavStatus.waypoint_current_distance = sqrt(
        pow(stod(waypoint_csv[waypoint_index][0]) - WaypointNavStatus.robot_pose.position.x, 2) +
        pow(stod(waypoint_csv[waypoint_index][1]) - WaypointNavStatus.robot_pose.position.y, 2));

    if (WaypointNavStatus.waypoint_current_distance <= WaypointNavStatus.waypoint_radius_threshold)
    {
      ROS_INFO("Invade WayPoint Area ");
      return true;
    }
  }
  return false;
}

void WaypointServer::getRobotPose(tf2_ros::Buffer &tf_,
                                  raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
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
    WaypointNavStatus.robot_pose.position.x = global_pose.pose.position.x;
    WaypointNavStatus.robot_pose.position.y = global_pose.pose.position.y;
  }
  catch (tf2::LookupException &ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
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

bool WaypointServer::checkGoalReach(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                                    int &waypoint_index_)
{
  if (WaypointNavStatus.goal_reached_flag)
  {
    ROS_INFO("Goal Reached");
    ROS_INFO("Restart");
    waypoint_index_++;
    return true;
  }
  return false;
}

void WaypointServer::ModeFlagOff(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  WaypointNavStatus.next_waypoint_mode = false;
  WaypointNavStatus.final_goal_waypoint_mode = false;
  WaypointNavStatus.restart_waypoint_mode = false;
  WaypointNavStatus.goal_reached_mode = false;
  WaypointNavStatus.slope_obstacle_avoidance_mode = false;

  WaypointNavStatus.goal_reached_flag = false;
  WaypointNavStatus.restart_flag = false;
}

void WaypointServer::managementWaypointInfo(
    vector<vector<string>> &waypoint_csv, int &waypoint_csv_index, int &waypoint_index,
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus, ros::Publisher &way_passed,
    ros::Publisher &way_finish, ros::Publisher &way_mode_slope)
{
  if (waypoint_csv[waypoint_index].size() >= 0 && waypoint_csv[waypoint_index].size() <= 4)
  {
    ModeFlagOff(WaypointNavStatus);
    WaypointNavStatus.next_waypoint_mode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "Goal")
  {
    WaypointNavStatus.final_goal_waypoint_mode = true;
    if (waypoint_index == waypoint_csv_index && WaypointNavStatus.goal_reached_flag &&
        checkWaypointArea(WaypointNavStatus, waypoint_csv, waypoint_index, way_passed))
      WaypointNavStatus.final_goal_flag = true;

    if (waypoint_index == waypoint_csv_index) ModeFlagOff(WaypointNavStatus);

    if (WaypointNavStatus.final_goal_flag)
    {
      ROS_INFO("Final Goal Reached");
      ROS_INFO("Please ' Ctl + c ' ");
      std_msgs::Bool data;
      way_finish.publish(data);
      exit(0);
    }
  }
  else if (waypoint_csv[waypoint_index][4] == "GoalReStart")
  {
    ModeFlagOff(WaypointNavStatus);
    WaypointNavStatus.restart_waypoint_mode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "GoalReach")
  {
    ModeFlagOff(WaypointNavStatus);
    WaypointNavStatus.goal_reached_mode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "SlopeObstacleAvoidance")
  {
    if (WaypointNavStatus.next_waypoint_mode)
    {
      ros::Duration duration(3.0);
      duration.sleep();
      std_msgs::Bool data;
      way_mode_slope.publish(data);
    }
    ModeFlagOff(WaypointNavStatus);
    WaypointNavStatus.slope_obstacle_avoidance_mode = true;
  }
}

void WaypointServer::debug(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                           int &waypoint_index)
{
  cout << "______________________________________\n"
       << "|NextWaypointMode            : "
       << static_cast<bool>(WaypointNavStatus.next_waypoint_mode) << "      |\n"
       << "|FinalGoalWaypointMode       : " << static_cast<bool>(WaypointNavStatus.final_goal_flag)
       << "      |\n"
       << "|ReStartWaypointMode         : "
       << static_cast<bool>(WaypointNavStatus.restart_waypoint_mode) << "      |\n"
       << "|GoalReachedMode             : "
       << static_cast<bool>(WaypointNavStatus.goal_reached_mode) << "      |\n"
       << "|GoalReachedFlag             : "
       << static_cast<bool>(WaypointNavStatus.goal_reached_flag) << "      |\n"
       << "|SlopeObstacleAvoidanceMode  : "
       << static_cast<bool>(WaypointNavStatus.slope_obstacle_avoidance_mode) << "      |\n"
       << "|SlopeObstacleAvoidanceFlag  : "
       << static_cast<bool>(WaypointNavStatus.slope_obstacle_avoidance_flag) << "      |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointID                 : " << static_cast<bool>(waypoint_index) << "       |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointDistance            : " << WaypointNavStatus.waypoint_current_distance
       << "      |\n"
       << "|WaypointThreshold           : " << WaypointNavStatus.waypoint_radius_threshold
       << "      |\n"
       << "_______________________________________\n";
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
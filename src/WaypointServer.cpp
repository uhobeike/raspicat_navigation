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

bool WaypointServer::checkWaypointArea(bool &NextWaypointMode, bool &SlopeObstacleAvoidanceMode,
                                       float &waypoint_area_check,
                                       vector<vector<string>> &waypoint_csv, int &waypoint_index,
                                       vector<double> &robot_pose, float &waypoint_area_threshold,
                                       string &node_name, ros::Publisher &way_sound)
{
  if (NextWaypointMode || SlopeObstacleAvoidanceMode)
  {
    waypoint_area_check = sqrt(pow(stod(waypoint_csv[waypoint_index][0]) - robot_pose[0], 2) +
                               pow(stod(waypoint_csv[waypoint_index][1]) - robot_pose[1], 2));

    if (waypoint_area_check <= waypoint_area_threshold)
    {
      ROS_INFO("%s: WayPoint Passing", node_name.c_str());
      ROS_INFO("%s: Next Move Plan", node_name.c_str());
      waypoint_index++;
      std_msgs::Bool data;
      way_sound.publish(data);
      return true;
    }
  }
  else if (!NextWaypointMode)
  {
    waypoint_area_check = sqrt(pow(stod(waypoint_csv[waypoint_index][0]) - robot_pose[0], 2) +
                               pow(stod(waypoint_csv[waypoint_index][1]) - robot_pose[1], 2));

    if (waypoint_area_check <= waypoint_area_threshold)
    {
      ROS_INFO("%s: Invade WayPoint Area ", node_name.c_str());
      return true;
    }
  }
  return false;
}

void WaypointServer::getRobotPose(tf2_ros::Buffer &tf_, vector<double> &robot_pose_)
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

bool WaypointServer::checkGoalReach(bool &GoalReachedFlag_, string &node_name_,
                                    int &waypoint_index_)
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

void WaypointServer::ModeFlagOff(bool &NextWaypointMode, bool &FinalGoalWaypointMode,
                                 bool &ReStartWaypointMode, bool &GoalReachedMode,
                                 bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag,
                                 bool &GoalReachedFlag)
{
  NextWaypointMode = false;
  FinalGoalWaypointMode = false;
  ReStartWaypointMode = false;
  GoalReachedMode = false;
  SlopeObstacleAvoidanceMode = false;

  GoalReachedFlag = false;
  ReStartFlag = false;
}

void WaypointServer::managementWaypointInfo(
    vector<vector<string>> &waypoint_csv, int &waypoint_csv_index, int &waypoint_index,
    string &node_name, bool &NextWaypointMode, bool &FinalGoalWaypointMode,
    bool &ReStartWaypointMode, bool &GoalReachedMode, bool &GoalReachedFlag,
    bool &SlopeObstacleAvoidanceMode, bool &ReStartFlag, bool &FinalGoalFlag,
    float &waypoint_area_check, vector<double> &robot_pose, float &waypoint_area_threshold,
    ros::Publisher &way_sound, ros::Publisher &way_finish, ros::Publisher &way_mode_slope)
{
  if (waypoint_csv[waypoint_index].size() >= 0 && waypoint_csv[waypoint_index].size() <= 4)
  {
    ModeFlagOff(NextWaypointMode, FinalGoalWaypointMode, ReStartWaypointMode, GoalReachedMode,
                SlopeObstacleAvoidanceMode, ReStartFlag, GoalReachedFlag);
    NextWaypointMode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "Goal")
  {
    FinalGoalWaypointMode = true;
    if (waypoint_index == waypoint_csv_index && GoalReachedFlag &&
        checkWaypointArea(NextWaypointMode, SlopeObstacleAvoidanceMode, waypoint_area_check,
                          waypoint_csv, waypoint_index, robot_pose, waypoint_area_threshold,
                          node_name, way_sound))
      FinalGoalFlag = true;
    if (waypoint_index == waypoint_csv_index)
      ModeFlagOff(NextWaypointMode, FinalGoalWaypointMode, ReStartWaypointMode, GoalReachedMode,
                  SlopeObstacleAvoidanceMode, ReStartFlag, GoalReachedFlag);
    if (FinalGoalFlag)
    {
      ROS_INFO("%s: Final Goal Reached", node_name.c_str());
      ROS_INFO("%s: Please ' Ctl + c ' ", node_name.c_str());
      std_msgs::Bool data;
      way_finish.publish(data);
      exit(0);
    }
  }
  else if (waypoint_csv[waypoint_index][4] == "GoalReStart")
  {
    ModeFlagOff(NextWaypointMode, FinalGoalWaypointMode, ReStartWaypointMode, GoalReachedMode,
                SlopeObstacleAvoidanceMode, ReStartFlag, GoalReachedFlag);
    ReStartWaypointMode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "GoalReach")
  {
    ModeFlagOff(NextWaypointMode, FinalGoalWaypointMode, ReStartWaypointMode, GoalReachedMode,
                SlopeObstacleAvoidanceMode, ReStartFlag, GoalReachedFlag);
    GoalReachedMode = true;
  }
  else if (waypoint_csv[waypoint_index][4] == "SlopeObstacleAvoidance")
  {
    if (NextWaypointMode)
    {
      ros::Duration duration(3.0);
      duration.sleep();
      std_msgs::Bool data;
      way_mode_slope.publish(data);
    }
    ModeFlagOff(NextWaypointMode, FinalGoalWaypointMode, ReStartWaypointMode, GoalReachedMode,
                SlopeObstacleAvoidanceMode, ReStartFlag, GoalReachedFlag);
    SlopeObstacleAvoidanceMode = true;
  }
}

void WaypointServer::debug(bool &NextWaypointMode, bool &FinalGoalWaypointMode,
                           bool &ReStartWaypointMode, bool &GoalReachedMode, bool &GoalReachedFlag,
                           bool &SlopeObstacleAvoidanceMode, bool &SlopeObstacleAvoidanceFlag,
                           int &waypoint_index, float &waypoint_area_check,
                           float &waypoint_area_threshold)
{
  cout << "_______________________________________\n"
       << "|NextWaypointMode            : " << NextWaypointMode << "      |\n"
       << "|FinalGoalWaypointMode       : " << FinalGoalWaypointMode << "      |\n"
       << "|ReStartWaypointMode         : " << ReStartWaypointMode << "      |\n"
       << "|GoalReachedMode             : " << GoalReachedMode << "      |\n"
       << "|GoalReachedFlag             : " << GoalReachedFlag << "      |\n"
       << "|SlopeObstacleAvoidanceMode  : " << SlopeObstacleAvoidanceMode << "      |\n"
       << "|SlopeObstacleAvoidanceFlag  : " << SlopeObstacleAvoidanceFlag << "      |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointIndex               : " << waypoint_index << "      |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointDistance            : " << waypoint_area_check << "|\n"
       << "|WaypointThreshold           : " << waypoint_area_threshold << "      |\n"
       << "_______________________________________\n";
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
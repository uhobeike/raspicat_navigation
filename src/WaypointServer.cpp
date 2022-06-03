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

#include "raspicat_navigation/WaypointServer.hpp"

namespace raspicat_navigation
{
void WaypointServer::initialize(std::string name)
{
  ROS_INFO("raspicat_navigation::WaypointServer initialize");
}
void WaypointServer::run() { ROS_INFO("raspicat_navigation::WaypointServer run"); }

void WaypointServer::checkWaypointYmal(ros::NodeHandle &pnh)
{
  XmlRpc::XmlRpcValue waypoint_yaml;
  pnh.getParam("waypoints", waypoint_yaml);
  for (auto i = 0; i < waypoint_yaml.size(); ++i)
  {
    cout << waypoint_yaml[i]["position"]["x"] << "\n";
    cout << waypoint_yaml[i]["position"]["y"] << "\n";
    cout << waypoint_yaml[i]["euler_angle"]["z"] << "\n";

    for (auto j = 0; j < waypoint_yaml[i]["properties"].size(); ++j)
    {
      cout << (waypoint_yaml[i]["properties"][j]["function"]) << "\n";

      if (waypoint_yaml[i]["properties"][j]["function"] == "attention_speak")
      {
        cout << waypoint_yaml[i]["properties"][j]["speak_interval"] << "\n";
      }

      else if (waypoint_yaml[i]["properties"][j]["function"] == "slop")
      {
        cout << waypoint_yaml[i]["properties"][j]["scan_range_limit"] << "\n";
      }

      else if (waypoint_yaml[i]["properties"][j]["function"] == "step")
      {
        cout << waypoint_yaml[i]["properties"][j]["approach_run"] << "\n";
        cout << waypoint_yaml[i]["properties"][j]["power"] << "\n";
      }

      else if (waypoint_yaml[i]["properties"][j]["function"] == "variable_speed")
      {
        cout << waypoint_yaml[i]["properties"][j]["linear"] << "\n";
      }

      else if (waypoint_yaml[i]["properties"][j]["function"] == "variable_waypoint_radius")
      {
        cout << waypoint_yaml[i]["properties"][j]["radius"] << "\n";
      }
    }
  }
}

void WaypointServer::loadWaypointYmal(ros::NodeHandle &pnh, XmlRpc::XmlRpcValue &waypoint_yaml)
{
  pnh.getParam("waypoints", waypoint_yaml);
}

void WaypointServer::setWaypoint(
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base,
    move_base_msgs::MoveBaseGoal &goal, XmlRpc::XmlRpcValue &waypoint_yaml,
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x =
      static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["x"]);
  goal.target_pose.pose.position.y =
      static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["y"]);

  tf2::Quaternion q;

  q.setRPY(0, 0,
           static_cast<double>(
               waypoint_yaml[WaypointNavStatus.waypoint_current_id]["euler_angle"]["z"]));

  goal.target_pose.pose.orientation.z = q.getZ();
  goal.target_pose.pose.orientation.w = q.getW();

  ac_move_base.sendGoal(goal);
}

void WaypointServer::setNextWaypoint(
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac_move_base,
    move_base_msgs::MoveBaseGoal &goal, XmlRpc::XmlRpcValue &waypoint_yaml,
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  ROS_INFO("Increment Waypoint Current ID");
  WaypointNavStatus.waypoint_current_id++;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x =
      static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["x"]);
  goal.target_pose.pose.position.y =
      static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["y"]);

  tf2::Quaternion q;
  q.setRPY(0, 0,
           static_cast<double>(
               waypoint_yaml[WaypointNavStatus.waypoint_current_id]["euler_angle"]["z"]));

  goal.target_pose.pose.orientation.z = q.getZ();
  goal.target_pose.pose.orientation.w = q.getW();

  ac_move_base.sendGoal(goal);
}

bool WaypointServer::checkWaypointArea(
    XmlRpc::XmlRpcValue &waypoint_yaml,
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus, ros::Publisher &way_passed)
{
  WaypointNavStatus.waypoint_current_distance =
      sqrt(pow(static_cast<double>(
                   waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["x"]) -
                   WaypointNavStatus.robot_pose.position.x,
               2) +
           pow(static_cast<double>(
                   waypoint_yaml[WaypointNavStatus.waypoint_current_id]["position"]["y"]) -
                   WaypointNavStatus.robot_pose.position.y,
               2));

  if (WaypointNavStatus.waypoint_current_distance <= WaypointNavStatus.waypoint_radius_threshold)
  {
    std_msgs::Empty data;
    way_passed.publish(data);
    ROS_INFO("WayPoint Passing");
    return true;
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

bool WaypointServer::checkGoalReach(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  if (WaypointNavStatus.flags.goal_reach)
  {
    ROS_INFO("Goal Reached");
    return true;
  }
  return false;
}

void WaypointServer::eraseTimer(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus,
                                std::map<std::string, ros::Timer> &timer_for_function)
{
  if (not WaypointNavStatus.functions.attention_speak.function)
    timer_for_function.erase("speak_attention");
}

void WaypointServer::setFalseWaypointFunction(
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  raspicat_navigation_msgs::WaypointNavStatus setFalse;
  WaypointNavStatus.functions = setFalse.functions;
}

void WaypointServer::setFalseWaypointFlag(
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus, bool allFalse)
{
  if (not WaypointNavStatus.functions.goal.function) WaypointNavStatus.flags.goal_reach = false;
  if (not WaypointNavStatus.functions.stop.function) WaypointNavStatus.flags.restart = false;

  if (allFalse)
  {
    raspicat_navigation_msgs::WaypointNavStatus setFalse;
    WaypointNavStatus.flags = setFalse.flags;
  }
}

void WaypointServer::setWaypointFunction(
    dynamic_reconfigure::Client<dwa_local_planner::DWAPlannerConfig> &dynamic_reconfigure_client,
    XmlRpc::XmlRpcValue &waypoint_yaml,
    raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  if (waypoint_yaml[WaypointNavStatus.waypoint_current_id].hasMember("properties"))
  {
    for (auto i = 0; i < waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"].size();
         ++i)
    {
      if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
          "attention_speak")
      {
        WaypointNavStatus.functions.attention_speak.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "goal")
      {
        WaypointNavStatus.functions.goal.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "loop")
      {
        WaypointNavStatus.functions.loop.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "slop")
      {
        WaypointNavStatus.functions.slop.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "step")
      {
        WaypointNavStatus.functions.step.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "stop")
      {
        WaypointNavStatus.functions.stop.function = true;
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "variable_speed")
      {
        WaypointNavStatus.functions.variable_speed.function = true;
        if (not static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]
                                                 ["properties"][i]["vel_trans"]) == 0)
        {
          dwa_local_planner::DWAPlannerConfig config;
          dynamic_reconfigure_client.getCurrentConfiguration(config);
          config.max_vel_trans = static_cast<double>(
              waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["vel_trans"]);
          config.max_vel_x = static_cast<double>(
              waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["vel_trans"]);
          dynamic_reconfigure_client.setConfiguration(config);
        }
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "variable_waypoint_radius")
      {
        WaypointNavStatus.functions.variable_waypoint_radius.function = true;
        if (not static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]
                                                 ["properties"][i]["waypoint_radius"]) == 0)
          WaypointNavStatus.waypoint_radius_threshold =
              static_cast<double>(waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"]
                                               [i]["waypoint_radius"]);
      }

      else if (waypoint_yaml[WaypointNavStatus.waypoint_current_id]["properties"][i]["function"] ==
               "waiting_line")
      {
        WaypointNavStatus.functions.waiting_line.function = true;
      }
    }
  }
  if (not(WaypointNavStatus.functions.goal.function | WaypointNavStatus.functions.loop.function |
          WaypointNavStatus.functions.waiting_line.function |
          WaypointNavStatus.functions.stop.function))
  {
    WaypointNavStatus.functions.next_waypoint.function = true;
  }
}

void WaypointServer::debug(raspicat_navigation_msgs::WaypointNavStatus &WaypointNavStatus)
{
  cout << "______________________________________\n"
       << "|NextWaypoinFunction            : "
       << static_cast<bool>(WaypointNavStatus.functions.next_waypoint.function) << "      |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|GoalReachFlag            : " << static_cast<bool>(WaypointNavStatus.flags.goal_reach)
       << "      |\n"
       << "|ReStartFlag            : " << static_cast<bool>(WaypointNavStatus.flags.restart)
       << "      |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointID                 : " << WaypointNavStatus.waypoint_current_id + 1 << " |\n"
       << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
       << "|WaypointDistance            : " << WaypointNavStatus.waypoint_current_distance
       << "      |\n"
       << "|WaypointThreshold           : " << WaypointNavStatus.waypoint_radius_threshold
       << "      |\n"
       << "_______________________________________\n";
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
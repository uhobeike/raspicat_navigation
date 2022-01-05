#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

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
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointServer, raspicat_navigation::BaseWaypointServer)
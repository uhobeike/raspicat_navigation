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

#ifndef BASE_WAYPOINT_HELPER_PLUGIN_HPP_
#define BASE_WAYPOINT_HELPER_PLUGIN_HPP_

#include <ros/ros.h>

using namespace ::std;

namespace raspicat_navigation
{
class BaseWaypointServer
{
 public:
  virtual void initialize(std::string name) = 0;
  virtual void run() = 0;
  virtual void WaypointCsvRead(string &csv_fname_, vector<vector<string>> &waypoint_csv_,
                               int &waypoint_csv_index_) = 0;

  virtual ~BaseWaypointServer() {}

 protected:
  BaseWaypointServer() {}
};

}  // namespace raspicat_navigation
#endif  // BASE_WAYPOINT_HELPER_PLUGIN_HPP_
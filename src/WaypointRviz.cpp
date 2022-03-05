#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

#include "raspicat_navigation/BaseWaypointRviz.hpp"

using namespace ::std;

namespace raspicat_navigation
{
class WaypointRviz : public raspicat_navigation::BaseWaypointRviz
{
 public:
  void initialize(std::string name) { ROS_INFO("raspicat_navigation::WaypointRviz initialize"); }
  void run() { ROS_INFO("raspicat_navigation::WaypointRviz run"); }

  void WaypointRvizVisualization(vector<vector<string>>& waypoint_csv_, int& waypoint_csv_index_,
                                 ros::Publisher& way_pose_array_, ros::Publisher& way_area_array_,
                                 ros::Publisher& way_number_txt_array_,
                                 float& waypoint_area_threshold_)
  {
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    visualization_msgs::MarkerArray waypoint_area;
    visualization_msgs::MarkerArray waypoint_number_txt;
    waypoint_area.markers.resize(waypoint_csv_index_ + 1);
    waypoint_number_txt.markers.resize(waypoint_csv_index_ + 1);
    uint8_t vec_cnt_index(0);
    for (auto it_t = waypoint_csv_.begin(); it_t != waypoint_csv_.end(); ++it_t)
    {
      vec_cnt_index = 0;
      WaypointMarkerArraySet(waypoint_area, waypoint_number_txt,
                             distance(waypoint_csv_.begin(), it_t),
                             waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size(),
                             waypoint_area_threshold_, waypoint_csv_);

      for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it)
      {
        switch (vec_cnt_index)
        {
          case 0:
            pose.position.x = stod(*it);
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
              waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x =
                  stod(*it);
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() > 4)
              if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)][4] ==
                  "SlopeObstacleAvoidance")
                waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x =
                    stod(*it);
            waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.x =
                stod(*it);
            vec_cnt_index++;
            continue;
          case 1:
            pose.position.y = stod(*it);
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
              waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y =
                  stod(*it);
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() > 4)
              if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)][4] ==
                  "SlopeObstacleAvoidance")
                waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y =
                    stod(*it);
            waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.y =
                stod(*it);
            vec_cnt_index++;
            continue;

            pose.position.z = 0.2;
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() == 4)
              waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z =
                  stod(*it);
            if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)].size() > 4)
              if (waypoint_csv_[distance(waypoint_csv_.begin(), it_t)][4] ==
                  "SlopeObstacleAvoidance")
                waypoint_area.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z =
                    stod(*it);
            waypoint_number_txt.markers[distance(waypoint_csv_.begin(), it_t)].pose.position.z =
                0.1;
          case 2:
            pose.orientation.z = stod(*it);
            vec_cnt_index++;
            continue;
          case 3:
            pose.orientation.w = stod(*it);
            vec_cnt_index++;
            break;

          default:
            break;
        }
      }
      pose_array.poses.push_back(pose);
    }
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";

    way_pose_array_.publish(pose_array);
    way_area_array_.publish(waypoint_area);
    way_number_txt_array_.publish(waypoint_number_txt);
  }

  void WaypointMarkerArraySet(visualization_msgs::MarkerArray& waypoint_area,
                              visualization_msgs::MarkerArray& waypoint_number_txt, uint8_t index,
                              uint8_t size, float& waypoint_area_threshold_,
                              vector<vector<string>>& waypoint_csv_)
  {
    /*waypoint area_________________________________________________________*/
    waypoint_area.markers[index].header.frame_id = "map";
    waypoint_area.markers[index].header.stamp = ros::Time::now();
    waypoint_area.markers[index].id = index;
    waypoint_area.markers[index].type = visualization_msgs::Marker::CYLINDER;
    waypoint_area.markers[index].action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 cylinder;
    cylinder.x = waypoint_area_threshold_ * 2;
    cylinder.y = waypoint_area_threshold_ * 2;
    cylinder.z = 0.03;
    waypoint_area.markers[index].scale = cylinder;
    if (size == 4)
      waypoint_area.markers[index].color.a = 0.1f;
    else
    {
      if (waypoint_csv_[index][4] == "SlopeObstacleAvoidance")
        waypoint_area.markers[index].color.a = 0.1f;
      else
        waypoint_area.markers[index].color.a = 0.000001f;
    }
    waypoint_area.markers[index].color.b = 1.0f;
    waypoint_area.markers[index].color.g = 0.0f;
    waypoint_area.markers[index].color.r = 0.0f;
    waypoint_area.markers[index].pose.orientation.z = 0;
    waypoint_area.markers[index].pose.orientation.w = 1;
    /*______________________________________________________________________*/

    /*waypoint_number_txt________________________________________________________________*/
    waypoint_number_txt.markers[index].header.frame_id = "map";
    waypoint_number_txt.markers[index].header.stamp = ros::Time::now();
    waypoint_number_txt.markers[index].id = index;
    waypoint_number_txt.markers[index].text = to_string(index + 1);
    waypoint_number_txt.markers[index].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    waypoint_number_txt.markers[index].action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 text;
    text.z = waypoint_area_threshold_ / 2;
    waypoint_number_txt.markers[index].scale = text;
    waypoint_number_txt.markers[index].color.a = 1.0f;
    waypoint_number_txt.markers[index].color.b = 1.0f;
    waypoint_number_txt.markers[index].color.g = 1.0f;
    waypoint_number_txt.markers[index].color.r = 1.0f;
    /*___________________________________________________________________________________*/
  }
};
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointRviz, raspicat_navigation::BaseWaypointRviz)
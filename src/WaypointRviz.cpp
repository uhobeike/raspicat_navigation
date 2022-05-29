
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include "raspicat_navigation/WaypointRviz.hpp"

namespace raspicat_navigation
{
void WaypointRviz::initialize(std::string name)
{
  ROS_INFO("raspicat_navigation::WaypointRviz initialize");
}
void WaypointRviz::run() { ROS_INFO("raspicat_navigation::WaypointRviz run"); }

void WaypointRviz::WaypointRvizVisualization(XmlRpc::XmlRpcValue& waypoint_yaml,
                                             ros::Publisher& way_pose_array,
                                             ros::Publisher& way_area_array,
                                             ros::Publisher& way_number_txt_array,
                                             float& waypoint_area_threshold)
{
  geometry_msgs::PoseArray waypoint_pose_array;
  geometry_msgs::Pose waypoint_pose;
  visualization_msgs::MarkerArray waypoint_area;
  visualization_msgs::MarkerArray waypoint_number_txt;
  waypoint_area.markers.resize(waypoint_yaml.size());
  waypoint_number_txt.markers.resize(waypoint_yaml.size());

  for (auto waypoint_id = 0; waypoint_id < waypoint_yaml.size(); ++waypoint_id)
  {
    WaypointRviz::WaypointMarkerArraySet(waypoint_yaml, waypoint_pose, waypoint_area,
                                         waypoint_number_txt, waypoint_id, waypoint_area_threshold);

    waypoint_pose_array.poses.push_back(waypoint_pose);
  }
  waypoint_pose_array.header.stamp = ros::Time::now();
  waypoint_pose_array.header.frame_id = "map";

  way_pose_array.publish(waypoint_pose_array);
  way_area_array.publish(waypoint_area);
  way_number_txt_array.publish(waypoint_number_txt);
}

void WaypointRviz::WaypointMarkerArraySet(XmlRpc::XmlRpcValue& waypoint_yaml,
                                          geometry_msgs::Pose& waypoint_pose,
                                          visualization_msgs::MarkerArray& waypoint_area,
                                          visualization_msgs::MarkerArray& waypoint_number_txt,
                                          uint8_t waypoint_id, float& waypoint_area_threshold)
{
  /*waypoint pose_________________________________________________________*/
  waypoint_pose.position.x = static_cast<double>(waypoint_yaml[waypoint_id]["position"]["x"]);
  waypoint_pose.position.y = static_cast<double>(waypoint_yaml[waypoint_id]["position"]["y"]);

  tf2::Quaternion q;
  q.setRPY(0, 0, static_cast<double>(waypoint_yaml[waypoint_id]["euler_angle"]["z"]));
  waypoint_pose.orientation.z = q.getZ();
  waypoint_pose.orientation.w = q.getW();

  waypoint_pose.position.z = 0.2;
  /*______________________________________________________________________*/

  /*waypoint area_________________________________________________________*/
  waypoint_area.markers[waypoint_id].header.frame_id = "map";
  waypoint_area.markers[waypoint_id].header.stamp = ros::Time::now();
  waypoint_area.markers[waypoint_id].id = waypoint_id;
  waypoint_area.markers[waypoint_id].pose = waypoint_pose;
  waypoint_area.markers[waypoint_id].type = visualization_msgs::Marker::CYLINDER;
  waypoint_area.markers[waypoint_id].action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 cylinder;
  cylinder.x = waypoint_area_threshold * 2;
  cylinder.y = waypoint_area_threshold * 2;
  cylinder.z = 0.03;
  waypoint_area.markers[waypoint_id].scale = cylinder;

  for (auto i = 0; i < waypoint_yaml[waypoint_id]["properties"].size(); i++)
  {
    if (!(waypoint_yaml[waypoint_id]["properties"][i]["function"] == "goal"))
      waypoint_area.markers[waypoint_id].color.a = 0.1f;
    else
    {
      waypoint_area.markers[waypoint_id].color.a = 0.000001f;
      break;
    }
  }
  waypoint_area.markers[waypoint_id].color.b = 1.0f;
  waypoint_area.markers[waypoint_id].color.g = 0.0f;
  waypoint_area.markers[waypoint_id].color.r = 0.0f;
  waypoint_area.markers[waypoint_id].pose.orientation.z = 0;
  waypoint_area.markers[waypoint_id].pose.orientation.w = 1;
  /*______________________________________________________________________*/

  /*waypoint_number_txt________________________________________________________________*/
  waypoint_number_txt.markers[waypoint_id].header.frame_id = "map";
  waypoint_number_txt.markers[waypoint_id].header.stamp = ros::Time::now();
  waypoint_number_txt.markers[waypoint_id].id = waypoint_id;
  waypoint_number_txt.markers[waypoint_id].pose = waypoint_pose;
  waypoint_number_txt.markers[waypoint_id].pose.position.z = 0.1;
  waypoint_number_txt.markers[waypoint_id].text = to_string(waypoint_id + 1);
  waypoint_number_txt.markers[waypoint_id].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  waypoint_number_txt.markers[waypoint_id].action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 text;
  text.z = waypoint_area_threshold / 2;
  waypoint_number_txt.markers[waypoint_id].scale = text;
  waypoint_number_txt.markers[waypoint_id].color.a = 1.0f;
  waypoint_number_txt.markers[waypoint_id].color.b = 1.0f;
  waypoint_number_txt.markers[waypoint_id].color.g = 1.0f;
  waypoint_number_txt.markers[waypoint_id].color.r = 1.0f;
  /*___________________________________________________________________________________*/
}  // namespace raspicat_navigation
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaypointRviz, raspicat_navigation::BaseWaypointRviz)
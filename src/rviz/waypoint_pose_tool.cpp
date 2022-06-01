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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

#include "waypoint_pose_tool.h"

namespace rviz
{
void WaypointPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.7f, 0.5f, 1.0f, 1.0f);
  setName("2D Waypoint Set");
  updateTopic();
}

WaypointPoseTool::WaypointPoseTool()
{
  shortcut_key_ = 'w';

  topic_property_ =
      new StringProperty("Topic", "waypoint_set", "The topic on which to publish waypoint set.",
                         getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointPoseTool::updateTopic()
{
  try
  {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("WaypointPoseTool", e.what());
  }
}

void WaypointPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.orientation = tf2::toMsg(quat);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  ROS_INFO("Setting waypoint: %.3f %.3f %.5f %.5f [frame=%s]", x, y, pose.pose.orientation.y,
           pose.pose.orientation.z, fixed_frame.c_str());
  pub_.publish(pose);
}

}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointPoseTool, rviz::Tool)
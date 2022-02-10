#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_navigation/CmdVelSmoother.hpp"

namespace raspicat_navigation
{
void CmdVelSmoother::readParameters()
{
  pnh_.param("acc_limit_x", acc_limit_x_, 1.0);
  pnh_.param("acc_limit_z", acc_limit_z_, 2.0);
  pnh_.param("decel_factor_x", dec_fac_x_, 1.3);
  pnh_.param("decel_factor_z", dec_fac_z_, 1.3);
}

void CmdVelSmoother::initPubSub()
{
  cmd_vel_publisher_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  cmd_vel_subscriber_ = pnh_.subscribe<geometry_msgs::Twist>("/nav_vel", 1, [&](auto &msg) {
    latest_cmdvel_time_ = ros::Time::now();
    ROS_DEBUG("Receive /nav_vel");
    geometry_msgs::Twist vel;
    vel = *msg;
    cmd_vel_que_.push_back(vel);
    if (cmd_vel_que_.size() == 2)
    {
      calculateDeceleration((latest_cmdvel_time_ - oldest_cmdvel_time_).toSec());
      publishSmoothVelcity();
      ROS_DEBUG("Publish /cmd_vel");
      cmd_vel_que_.erase(cmd_vel_que_.begin());
    }
    oldest_cmdvel_time_ = ros::Time::now();
  });
}

void CmdVelSmoother::calculateDeceleration(double pass_sec)
{
  if (pass_sec == 0) return;
  double acc_x = (cmd_vel_que_[0].linear.x - cmd_vel_que_[1].linear.x) / pass_sec;
  double acc_z = (cmd_vel_que_[0].angular.z - cmd_vel_que_[1].angular.z) / pass_sec;

  publish_cmd_vel_ = cmd_vel_que_[1];

  if (std::abs(acc_x) > acc_limit_x_)
  {
    if (cmd_vel_que_[1].linear.x >= 0)
      publish_cmd_vel_.linear.x = cmd_vel_que_[0].linear.x / dec_fac_x_;
    ROS_DEBUG("before-linear.x:%f, after-linear.x:%f", cmd_vel_que_[1].linear.x,
              publish_cmd_vel_.linear.x);
    cmd_vel_que_[1] = publish_cmd_vel_;
  }
  if (std::abs(acc_z) > acc_limit_z_)
  {
    if (cmd_vel_que_[0].angular.z > cmd_vel_que_[1].angular.z)
      publish_cmd_vel_.angular.z = cmd_vel_que_[0].angular.z / dec_fac_z_;
    else if (cmd_vel_que_[0].angular.z < cmd_vel_que_[1].angular.z)
      publish_cmd_vel_.angular.z = cmd_vel_que_[1].angular.z / dec_fac_z_;
    ROS_DEBUG("before-angular.z:%f, after-angular.z:%f", cmd_vel_que_[1].angular.z,
              publish_cmd_vel_.angular.z);
    cmd_vel_que_[1] = publish_cmd_vel_;
  }
}

void CmdVelSmoother::publishSmoothVelcity() { cmd_vel_publisher_.publish(publish_cmd_vel_); }

}  // namespace raspicat_navigation

PLUGINLIB_EXPORT_CLASS(raspicat_navigation::CmdVelSmoother,
                       raspicat_navigation::WaypointNavHelperPlugin)
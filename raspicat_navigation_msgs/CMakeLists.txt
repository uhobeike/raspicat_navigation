cmake_minimum_required(VERSION 3.0.2)
project(raspicat_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Flag.msg
  Property.msg
  Waypoint.msg
  Waypoints.msg
  WaypointNavStatus.msg
)

add_message_files(
  DIRECTORY msg/function
  FILES
  attention_speak.msg
  clear_costmap.msg
  loop.msg
  goal.msg
  next_waypoint.msg
  slope.msg
  step.msg
  stop.msg
  variable_speed.msg
  variable_waypoint_radius.msg
  waiting_line.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime)
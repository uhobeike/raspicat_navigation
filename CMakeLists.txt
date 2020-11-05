################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(raspicat_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES goal_send
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

################################################################################
# Build
################################################################################
add_executable(waypoint_navigation src/waypoint_navigation.cpp)
target_link_libraries(waypoint_navigation ${catkin_LIBRARIES})

add_executable(waypoint_rviz src/waypoint_rviz_set.cpp)
target_link_libraries(waypoint_rviz ${catkin_LIBRARIES})

add_executable(start_order_waypoint_navigation src/start_order_waypoint_navigation.cpp)
target_link_libraries(start_order_waypoint_navigation ${catkin_LIBRARIES})

add_executable(waypoint_view src/waypoint_view.cpp)
target_link_libraries(waypoint_view ${catkin_LIBRARIES})
################################################################################
# Install
################################################################################
install(DIRECTORY launch src config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################

cmake_minimum_required(VERSION 2.8.3)
project(raspicat_navigation_rviz_plugin)

add_compile_options(-std=c++14)

link_libraries(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
  roslib
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/waypoint_pose_tool.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(DIRECTORY src 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_rviz.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
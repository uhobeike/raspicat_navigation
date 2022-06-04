#!/bin/bash -xve

# Check ros node process
top -n 1 -b | head -n 20
while true; do sleep 10 | top -n 1 -b | head -n 20; done &

# Gazebo launch
roslaunch raspicat_navigation raspicat_tsudanuma_2_19_world.launch \
  x_gazebo:=0.155971128532 y_gazebo:=-0.0254326737864 yaw_gazebo:=0 open_gui:=false &
sleep 20

# Rviz & Navigation launch
roslaunch raspicat_navigation ci_test.launch \
  mcl:=amcl waypoint_yaml_file:=$(rospack find raspicat_navigation)/test/waypoint.yaml \
  map_name:=tsudanuma_2_19 open_rviz:=false &
sleep 60

# Execute start operation
rostopic pub -1 /way_nav_start std_msgs/Empty

# Execute restart operation
timeout 300 $(rostopic echo -n 1 /waypoint_stop_function;rostopic pub -1 /way_nav_restart std_msgs/Empty)

# Check goal
timeout 300 rostopic echo -n 1 /waypoint_goal_function

# Printf result
if [ $? -eq 0 ];then 
  killall rosmaster
  printf '\033[42m%s\033[m\n' 'Docker Test SUCCEED'
  exit 0
else
  printf '\033[31m%s\033[m\n' 'Docker Test FAILED'
  exit 1
fi
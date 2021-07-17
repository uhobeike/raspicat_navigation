#!/bin/bash -xve

xvfb-run --auto-servernum -s "-screen 0 1400x900x24" roslaunch turtlebot3_gazebo turtlebot3_world.launch &
sleep 10

xvfb-run --auto-servernum -s '-screen 0 1400x900x24' roslaunch raspicat_navigation ci_test.launch &
sleep 10

rostopic  pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.862, y: -0.621, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.03052, w: 0.99953}"
sleep 5 

rostopic pub -1 /move_base/goal move_base_msgs/MoveBaseActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'map'
    pose:
      position:
        x: 1.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.5
        w: 1.0"

while true; do sleep 1 | rostopic echo -n 1 /move_base/feedback | grep -A 10 pose; done &

bash -c "timeout 40 echo $(rostopic echo -n 1 /move_base/result | grep "Goal reached")| sed 's/^.*"\(.*\)".*$/\1/'
# if [ $? -eq 0 ];then echo "a"; break;else echo "b";break;fi"

if [ $? -eq 0 ];then killall rosmaster && exit 0; break;else exit 1 ;break;fi
#!/bin/bash -xve

roslaunch turtlebot3_gazebo turtlebot3_world.launch &
sleep 10

roslaunch raspicat_navigation ci_test.launch &
sleep 10

rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
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
        x: 0.658029079437
        y: -0.549129128456
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.0150406081599
        w: 0.999886883655"

while true; do sleep 1 | rostopic echo -n 1 /move_base/feedback | grep -A 10 pose; done &

timeout 40 echo $(rostopic echo -n 1 /move_base/result | grep "Goal reached")| sed 's/^.*"\(.*\)".*$/\1/'

if [ $? -eq 0 ];then 
    killall rosmaster
    printf '\033[42m%s\033[m\n' 'DOCKER TEST SUCCEED'
    exit 0
    break
else
    printf '\033[31m%s\033[m\n' 'DOCKER TEST FAILED'
    exit 1
    break
fi
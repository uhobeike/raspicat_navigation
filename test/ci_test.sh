#!/bin/bash -xve
if [ $(whoami) = "runner" ]; then
    DOCKER_CONTAINER_ID=$(docker ps -a | grep "ros_entrypoint.sh" | awk '{print $1}')
    docker start $DOCKER_CONTAINER_ID

    docker exec $DOCKER_CONTAINER_ID /bin/bash -c \
        "source ~/.bashrc && \
        ./raspicat_navigation/test/ci_test.sh"
else 
    source ~/.bashrc
    export TURTLEBOT3_MODEL=burger
    (xvfb-run --auto-servernum -s '-screen 0 1400x900x24' roslaunch turtlebot3_gazebo turtlebot3_world.launch &)
    sleep 15
    killall rosmaster &
    exit 0
fi
# source /home/catkin_ws/devel/setup.bash;
# export TURTLEBOT3_MODEL=burger;
# (xvfb-run --auto-servernum -s '-screen 0 1400x900x24' roslaunch turtlebot3_gazebo turtlebot3_world.launch &); 
# sleep 15;
# killall rosmaster &;
# exit 0"

# (rostopic pub -1 /move_base/goal move_base_msgs/MoveBaseActionGoal "header: \
#   seq: 0 \
#   stamp: \
#     secs: 0 \
#     nsecs: 0 \
#   frame_id: '' \
# goal_id: \
#   stamp: \
#     secs: 0 \
#     nsecs: 0 \
#   id: '' \
# goal: \
#   target_pose: \
#     header: \
#       seq: 0 \
#       stamp: \
#         secs: 0 \
#         nsecs: 0 \
#       frame_id: 'map' \
#     pose: \
#       position: \
#         x: 1.0 \
#         y: 0.0 \
#         z: 0.0 \
#       orientation: \
#         x: 0.0 \
#         y: 0.0 \
#         z: 0.5 \
#         w: 1.0");
#         killall rosmaster &"
    # while true; do sleep 1 | rostopic echo -n 1 /move_base/feedback | grep -A 10 pose; done &;
    
    # bash -c "timeout 40 echo $(rostopic echo -n 1 /move_base/result | grep "Goal reached")| sed 's/^.*"\(.*\)".*$/\1/';
    # if [ $? -eq 0 ];then echo "a"; break;else echo "b";break;fi"
    
    # if [ $? -eq 0 ];then killall rosmaster; break;else exit 1 | exit 1;break;fi
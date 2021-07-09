#!/bin/bash -xve
DOCKER_CONTAINER_ID=$(docker ps -a | grep "ros_entrypoint.sh" | awk '{print $1}')
docker start $DOCKER_CONTAINER_ID
docker exec $DOCKER_CONTAINER_ID /bin/bash -c \
    "source /ros_entrypoint.sh;
    source /home/catkin_ws/devel/setup.bash;
    export TURTLEBOT3_MODEL=burger;
    (xvfb-run --auto-servernum  -s '-screen 0 1400x900x24' roslaunch turtlebot3_gazebo turtlebot3_world.launch &); 
    sleep 10;
    rostopic list;
    killall rosmaster"
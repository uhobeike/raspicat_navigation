#!/bin/bash -xve
DOCKER_CONTAINER_ID=$(docker ps -a | grep "ubeike/raspicat-ros1-melodic-navigation" | awk '{print $1}')

docker exec -it $DOCKER_CONTAINER_ID /bin/bash -c \
    "xvfb-run --auto-servernum  -s '-screen 0 1400x900x24' roslaunch turtlebot3_gazebo turtlebot3_world.launch;
    sleep 5"

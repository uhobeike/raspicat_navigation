#!/bin/bash -xve
DOCKER_CONTAINER_ID=$(docker ps -a | grep "ubeike/raspicat-ros1-melodic-navigation" | awk '{print $1}')
docker start $DOCKER_CONTAINER_ID
docker exec $DOCKER_CONTAINER_ID /bin/bash -c \
    "xvfb-run --auto-servernum  -s '-screen 0 1400x900x24' roscore&"

#!/bin/bash -xve
cd ..

docker run \
    -v $(pwd):/home/catkin_ws/src \
ubeike/raspicat-ros1-melodic-navigation /bin/bash -c \
    "git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git;
    git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git;
    git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git;
    rosdep install -r -y --from-paths --ignore-src .;
    cd /home/catkin_ws && catkin build && source /home/catkin_ws/devel/setup.bash"
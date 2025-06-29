# ros_motion_controller

## Prerequisites
- make sure docker is installed! see https://docs.docker.com/get-started/get-docker/
- written in cpp
- using ROS 2 Humble

## To Run
docker build -t my/ros:aptgetter .

docker run -it -v ~/Desktop/ros_motion_controller/src:/Workspace --rm my/ros:aptgetter /bin/bash

## Recurring commands
source /opt/ros/humble/setup.bash
cd Workspace
apt update
rosdep install -i --from-path src --rosdistro humble -y
colcon build

make a new terminal!

source /opt/ros/humble/setup.bash
cd Workspace
source install/local_setup.bash
ros2 run turtlesim turtlesim_node

#!/usr/bin/env bash

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${HOME}/ros2_ws/install/setup.bash"
BRINGUP="${HOME}/ros2_ws/src/gp8_ros2/gp8_bringup/scripts"

source "$ROS_SETUP"
source "$WS_SETUP"

cd "$BRINGUP" || exit 1

bash gp8_gazebo_and_moveit.sh

exec bash

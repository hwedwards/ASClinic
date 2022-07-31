#!/bin/bash
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/noetic/setup.bash
#source /home/asc-share/asclinic-system/catkin_ws/src/asclinic_pkg/launch/Config.sh
#
# Change directory to the dfall-system repository
cd /home/asc-share/asclinic-system/catkin_ws
#
# Call catkin_make
catkin_make 2>&1

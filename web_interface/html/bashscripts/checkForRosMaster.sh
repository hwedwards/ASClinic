#!/bin/bash
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/noetic/setup.bash
#
# Perform the  check
# > Note: the -q options converts the 
#   grep output to a true/false
if rosnode list | grep -q /rosout; then
    echo "ROS Master exists"
else
    echo "ROS Master not found"
fi

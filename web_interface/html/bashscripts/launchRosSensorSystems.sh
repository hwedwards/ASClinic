#!/bin/bash
#
# Check that exactly one command is supplied
if [ "$#" -ne 1 ]; then
	echo "exactly one argument must be supplied"
	exit 1
fi
#
# Check that the first command supplied is valid
if [ "$1" != "true" ] && [ "$1" != "false" ]; then
	echo "emulate i2c = $1, is not a valid boolean."
	exit 1
fi
#
# Put the commands into variables for make things more readable
emultei2c=$1
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/noetic/setup.bash
source /home/asc-share/asclinic-system/catkin_ws/devel/setup.bash
#source /home/asc-share/asclinic-system/catkin_ws/src/asclinic_pkg/launch/Config.sh
#
# Set the ROS log location
export ROS_LOG_DIR=/var/www/html/.ros/log
#
# Mount the catkin workspace folder
#sudo mount --bind /home/asc-share/asclinic-system/catkin_ws /var/www/html/catkin_ws
#
# Check that the ROS Master exists
# > Note: the -q options converts the
#   grep output to a true/false
if rosnode list | grep -q /rosout; then
	echo "ROS is already running"
else
	#nohup roslaunch asclinic_pkg sensor_systems.launch > /dev/null 2>&1 &
	nohup roslaunch asclinic_pkg sensor_systems.launch emulate_i2c:=$emultei2c > /dev/null 2>&1 &
	echo "ROS successfully launched"
fi

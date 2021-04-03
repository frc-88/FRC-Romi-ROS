#!/usr/bin/env bash
source /home/pi/ros_ws/devel/setup.bash
source /etc/systemd/system/ros/bin/env.sh

export ROS_HOME=/home/pi/.ros
export DISPLAY=:0
roslaunch tj2_romi_bringup tj2_romi_bringup.launch &
PID=$!
wait "$PID"

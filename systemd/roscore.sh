#!/usr/bin/env bash
/home/pi/fix_tmp.sh
source /opt/ros/melodic/setup.sh
source /etc/systemd/system/ros/bin/env.sh
roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done

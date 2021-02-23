#!/usr/bin/env bash

HOST_MACHINE=""
USE_HOTSPOT=0

stop_time=$((SECONDS+300))

while true; do
    if [ $USE_HOTSPOT = 1 ]; then
        HOST_MACHINE=`ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
        if [ ! -z ${HOST_MACHINE} ]; then
            break
        fi
    fi
    HOST_MACHINE=`ifconfig wlan1 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [ ! -z ${HOST_MACHINE} ]; then
        break
    fi
    sleep 0.5
    if [ $SECONDS -gt $stop_time ]; then
        break
    fi
done


export ROS_IP=${HOST_MACHINE}
export ROS_MASTER_URI=http://${HOST_MACHINE}:11311
# export ROS_MASTER_URI=http://${HOST_MACHINE_NAME}:11311

echo ${ROS_IP}
echo ${ROS_MASTER_URI}

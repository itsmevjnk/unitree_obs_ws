#!/bin/bash

if [ -z "${ROS_IP}" ]; then
    unset ROS_IP
fi

export ROS_MASTER_URI=http://192.168.123.161:11311
ros2 run ros1_bridge parameter_bridge

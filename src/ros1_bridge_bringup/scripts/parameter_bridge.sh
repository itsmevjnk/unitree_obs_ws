#!/bin/bash

if [ -z "${ROS_IP}" ]; then
    unset ROS_IP
fi

ros2 run ros1_bridge parameter_bridge

#!/bin/bash

ROBOT_NAME=$1
TRY_COUNT=1
ROBOT_IP=''
while [ -z "${ROBOT_IP}" ]; do
    echo "Pinging Duckiebot $ROBOT_NAME (attempt #$TRY_COUNT)..."
    ROBOT_IP=$(ping $ROBOT_NAME.local -c1 2>/dev/null | head -n1 | awk -F' ' '{print$3}' | tr -d '()')
    TRY_COUNT=$(($TRY_COUNT + 1))
done
echo "Robot IP address: $ROBOT_IP"

IF_NAME=$(ip route get $ROBOT_IP | head -n1 | awk -F' dev ' '{print$2}' | awk -F' ' '{print$1}')
LOCAL_IP=$(ip addr | grep "^.*inet.*$IF_NAME$" | awk -F' ' '{print$2}' | awk -F'/' '{print$1}')
echo "Interface used to contact robot: $IF_NAME ($LOCAL_IP)"

ROS_MASTER_URI=http://$ROBOT_IP:11311 ROS_IP=$LOCAL_IP ros2 run ros1_bridge parameter_bridge

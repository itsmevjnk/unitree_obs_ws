#!/bin/bash

if [ -z "${ROS_IP}" ]; then
    unset ROS_IP
fi

if [ -n "${CYCLONEDDS_URI}" ]; then
    # copy file specified by CYCLONEDDS_URI to somewhere safe, so we can mount it to the container
    TEMP_FILE=$(mktemp /tmp/cyclonedds_XXXXXX.xml)
    cp "${CYCLONEDDS_URI#file://}" "${TEMP_FILE}"
    CYCLONEDDS_ARG="-v ${TEMP_FILE}:/cyclonedds.xml -e CYCLONEDDS_URI=file:///cyclonedds.xml"
    echo "CycloneDDS configuration file copied to ${TEMP_FILE}"
else
    CYCLONEDDS_ARG=""
fi

echo "Launching ROS1 bridge."
docker run -it -e ROS_MASTER_URI -e RMW_IMPLEMENTATION $CYCLONEDDS_ARG --net=host $DOCKER_IMAGE


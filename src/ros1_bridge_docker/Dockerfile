FROM ros:galactic-ros1-bridge-focal

# install additional RMWs
RUN apt-get update && apt-get upgrade -y && apt-get install ros-galactic-rmw-*-cpp -y && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

# default command
CMD ["ros2", "run", "ros1_bridge", "parameter_bridge"]

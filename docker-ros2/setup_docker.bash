#!/bin/bash
echo "Setting up your docker..."
echo "Building the docker image..."
docker build -t prl_ros2:jazzy . --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
echo "Docker image built successfully."
echo "Creating share directory..."
mkdir -p share/ws/src
echo "Share directory created."
echo "Setting up the workspace..."
cd share/ws/src
cp -r ../../../../prl_ur5_* .
git clone -b ros2 https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
git clone -b ros2 https://github.com/inria-paris-robotics-lab/onrobot_ros.git
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
# git clone https://github.com/inria-paris-robotics-lab/wsg50-ros-pkg.git
echo "Workspace setup complete."

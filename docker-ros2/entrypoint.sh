#!/usr/bin/env bash
set -e
if [ ! -d "share/mantis_ws/src" ]; then
  echo "Missing mantis workspace, building it..."

  mkdir -p share/mantis_ws/src
  cd share/mantis_ws/src

  git clone --recursive https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git

  source /opt/ros/${ROS_DISTRO}/setup.bash
  vcs import < prl_ur5_ros2/dependencies.repos

  cd ..
  rosdep update
  rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
  colcon build --symlink-install
fi

exec "$@"

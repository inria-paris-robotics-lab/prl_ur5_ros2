#!/usr/bin/env bash
set -e

WS="$HOME/share/mantis_ws"
# Marker written only after a successful build: an interrupted/failed build is retried at next start
BUILT_MARKER="$WS/.mantis_ws_built"

if [ ! -f "$BUILT_MARKER" ]; then
  echo "Mantis workspace missing or not built, building it..."
  (
    set -e
    mkdir -p "$WS/src"
    cd "$WS/src"
    [ -d prl_ur5_ros2 ] || git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git

    source /opt/ros/${ROS_DISTRO}/setup.bash
    vcs import --skip-existing < prl_ur5_ros2/dependencies.repos

    cd "$WS"
    rosdep update
    rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
    colcon build --symlink-install
    touch "$BUILT_MARKER"
  ) || echo "WARNING: mantis workspace build failed, it will be retried at next container start (or rerun colcon build in $WS)"
fi

exec "$@"

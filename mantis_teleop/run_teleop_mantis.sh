#!/usr/bin/env bash
# Host launcher: start the Vive publisher and the Mantis teleop bridge together.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-$DIR/../SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash}"
HOST_DEPS="${HOST_DEPS:-$DIR/../mantis_host_deps/fakeprefix}"
source "$ROS_SETUP" 2>/dev/null
python -c "import rclpy, openvr, pinocchio, pink" || { echo "ROS env broken (imports failed)"; exit 1; }
export AMENT_PREFIX_PATH="$HOST_DEPS:$AMENT_PREFIX_PATH"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp_only.xml"

python "$DIR/vive_pub.py" &
VIVE_PID=$!
trap 'kill $VIVE_PID 2>/dev/null || true' EXIT

python "$DIR/teleop_mantis.py"

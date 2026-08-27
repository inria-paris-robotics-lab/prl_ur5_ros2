#!/usr/bin/env bash
# Publish HTC Vive controller pose/buttons to /vive/* (SteamVR backend).
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-$DIR/../SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash}"
source "$ROS_SETUP" 2>/dev/null
python -c "import rclpy, openvr" || { echo "ROS env broken (rclpy/openvr import failed)"; exit 1; }
export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp_only.xml"
python "$DIR/vive_pub.py"

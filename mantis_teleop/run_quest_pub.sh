#!/usr/bin/env bash
# Publish Quest controller pose/buttons to /vive/*; env defaults below are overridable.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-$DIR/../SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash}"
QUEST_DEPS="${QUEST_DEPS:-$DIR/../quest_deps}"
OCULUS_READER="${OCULUS_READER:-$DIR/../oculus_reader}"
export QUEST_BACKEND="${QUEST_BACKEND:-adb}"
export QUEST_ADB_MODE="${QUEST_ADB_MODE:-extrapolate}"
export QUEST_ROT_OFFSET="${QUEST_ROT_OFFSET:--90,0,0}"
source "$ROS_SETUP" 2>/dev/null

case "${QUEST_BACKEND:-openvr}" in
  adb|oculus_reader|usb)
    export PYTHONPATH="$QUEST_DEPS:$OCULUS_READER${PYTHONPATH:+:$PYTHONPATH}"
    python -c "import rclpy, numpy" || { echo "ROS env broken (rclpy/numpy import failed)"; exit 1; }
    python -c "import oculus_reader, ppadb" 2>/dev/null || {
      echo "oculus_reader/ppadb not importable. Set them up once:"
      echo "  git clone https://github.com/rail-berkeley/oculus_reader $OCULUS_READER"
      echo "  pip3 install --target $QUEST_DEPS pure-python-adb"
      echo "and check that 'adb devices' lists the headset as 'device' (not 'unauthorized')."
      exit 1; }
    command -v adb >/dev/null || { echo "adb not found: sudo apt install android-tools-adb"; exit 1; }
    ;;
  *)
    python -c "import rclpy, openvr" || { echo "ROS env broken (rclpy/openvr import failed)"; exit 1; }
    ;;
esac

export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp_only.xml"
python "$DIR/quest_pub.py" "$@"

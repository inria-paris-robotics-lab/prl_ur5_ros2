#!/usr/bin/env bash
# Host launcher: start the Quest publisher and the Mantis teleop bridge together.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-$DIR/../SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash}"
QUEST_DEPS="${QUEST_DEPS:-$DIR/../quest_deps}"
OCULUS_READER="${OCULUS_READER:-$DIR/../oculus_reader}"
HOST_DEPS="${HOST_DEPS:-$DIR/../mantis_host_deps/fakeprefix}"
source "$ROS_SETUP" 2>/dev/null
python -c "import rclpy, pinocchio, pink" || { echo "ROS env broken (imports failed)"; exit 1; }

case "${QUEST_BACKEND:-openvr}" in
  adb|oculus_reader|usb)
    export PYTHONPATH="$QUEST_DEPS:$OCULUS_READER${PYTHONPATH:+:$PYTHONPATH}"
    python -c "import oculus_reader, ppadb" 2>/dev/null || {
      echo "oculus_reader/ppadb not importable. Set them up once:"
      echo "  git clone https://github.com/rail-berkeley/oculus_reader $OCULUS_READER"
      echo "  pip3 install --target $QUEST_DEPS pure-python-adb"
      exit 1; }
    command -v adb >/dev/null || { echo "adb not found: sudo apt install android-tools-adb"; exit 1; }
    ;;
  *)
    python -c "import openvr" || { echo "openvr not importable (default backend needs SteamVR/ALVR; use QUEST_BACKEND=adb for USB)"; exit 1; }
    ;;
esac

export AMENT_PREFIX_PATH="$HOST_DEPS:$AMENT_PREFIX_PATH"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp_only.xml"

python "$DIR/quest_pub.py" "$@" &
QUEST_PID=$!
trap 'kill $QUEST_PID 2>/dev/null || true' EXIT

python "$DIR/teleop_mantis.py"

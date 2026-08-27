#!/usr/bin/env bash
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARE="${SHARE_DIR:-$HOME/rabota/docker_shared}"
WS="$SHARE/mantis_ws"
FP="${HOST_DEPS:-$HOME/rabota/mantis_host_deps/fakeprefix}"
FORCE_FILES=0
FORCE_PATCHES=0
CONTAINER="${CONTAINER_NAME:-mantis}"

for arg in "$@"; do
    case "$arg" in
        --force-files)   FORCE_FILES=1 ;;
        --force-patches) FORCE_PATCHES=1 ;;
        -h|--help)
            echo "usage: setup_workstation.sh [--force-files] [--force-patches]"
            echo
            echo "  SHARE_DIR      container share dir      (default ~/rabota/docker_shared)"
            echo "  HOST_DEPS      host package prefix      (default ~/rabota/mantis_host_deps/fakeprefix)"
            echo "  CONTAINER_NAME running container name   (default mantis)"
            echo
            echo "  --force-files    overwrite teleop files already in the share dir"
            echo "  --force-patches  re-apply the workspace patches over an existing mantis_ws"
            exit 0 ;;
        *) echo "unknown argument: $arg (see --help)"; exit 1 ;;
    esac
done

say() { printf '\n== %s\n' "$1"; }

say "share directory: $SHARE"
mkdir -p "$SHARE"

say "teleop files"
for f in teleop_mantis.py config_teleop_mantis.yaml quest_pub.py vive_pub.py \
         lerobot_recorder.py fastdds_udp_only.xml \
         run_quest_pub.sh run_teleop_quest.sh run_vive_pub.sh run_teleop_mantis.sh run_replay.sh; do
    if [[ -e "$SHARE/$f" && $FORCE_FILES -eq 0 ]]; then
        echo "  skip    $f (already there, --force-files to overwrite)"
    else
        cp -a "$SRC/$f" "$SHARE/$f"
        echo "  copied  $f"
    fi
done
if [[ -e "$SHARE/lerobot_robot_mantis" && $FORCE_FILES -eq 0 ]]; then
    echo "  skip    lerobot_robot_mantis/ (already there)"
else
    rm -rf "$SHARE/lerobot_robot_mantis"
    cp -a "$SRC/lerobot_robot_mantis" "$SHARE/"
    echo "  copied  lerobot_robot_mantis/"
fi

say "ROS workspace: $WS"
FRESH_WS=0
if [[ -d "$WS/src/prl_ur5_ros2" ]]; then
    echo "  already present, sources left untouched"
else
    mkdir -p "$WS/src"
    git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git "$WS/src/prl_ur5_ros2"
    if command -v vcs >/dev/null; then
        (cd "$WS/src" && vcs import . < prl_ur5_ros2/dependencies.repos)
    else
        echo "  vcstool not installed, cloning the dependency list directly"
        I=https://github.com/inria-paris-robotics-lab
        git clone -b ros2 $I/prl_ur5_robot_configuration.git "$WS/src/prl_ur5_robot_configuration"
        git clone -b ros2 $I/robotiq.git                     "$WS/src/robotiq"
        git clone -b ros2 $I/onrobot_ros.git                 "$WS/src/onrobot_ros"
        git clone       $I/wsg50-ros-pkg.git                 "$WS/src/wsg50-ros-pkg"
        git clone       $I/prl_ur5_calibration.git           "$WS/src/prl_ur5_calibration"
        git clone       $I/allegro_hand_ros_v4.git           "$WS/src/allegro_hand_ros_v4"
    fi
    FRESH_WS=1
    echo "  cloned prl_ur5_ros2 + its dependencies"
fi

say "workspace patches"
if [[ $FRESH_WS -eq 1 || $FORCE_PATCHES -eq 1 ]]; then
    cp -a "$SRC/patches/wsg50-ros-pkg/."               "$WS/src/wsg50-ros-pkg/"
    cp -a "$SRC/patches/prl_ur5_robot_configuration/." "$WS/src/prl_ur5_robot_configuration/"
    cp -a "$SRC/patches/prl_ur5_ros2/prl_ur5_gazebo/." "$WS/src/prl_ur5_ros2/prl_ur5_gazebo/"
    echo "  applied to wsg50-ros-pkg, prl_ur5_robot_configuration, prl_ur5_gazebo"
else
    echo "  workspace was already there, patches NOT applied (--force-patches to apply)"
fi

say "host package prefix: $FP"
IDX="$FP/share/ament_index/resource_index/packages"
mkdir -p "$IDX"

# Every package the mantis xacro resolves with $(find ...), linked out of the workspace
# sources. The ament index entry is what makes $(find ...) see them at all.
for entry in \
    "prl_ur5_description:prl_ur5_ros2/prl_ur5_description" \
    "prl_ur5_control:prl_ur5_ros2/prl_ur5_control" \
    "prl_ur5_robot_configuration:prl_ur5_robot_configuration" \
    "wsg_50_simulation:wsg50-ros-pkg/wsg_50_simulation" \
    "wsg_50_driver:wsg50-ros-pkg/wsg_50_driver" \
    "robotiq_ft_sensor_description:robotiq/robotiq_ft_sensor/robotiq_ft_sensor_description" \
    "allegro_hand_description:allegro_hand_ros_v4/allegro_hand_description" \
    "onrobot_description:onrobot_ros/onrobot_description" ; do
    pkg="${entry%%:*}"
    ln -sfn "$WS/src/${entry#*:}" "$FP/share/$pkg"
    touch "$IDX/$pkg"
done
echo "  linked 8 packages out of $WS/src"

# The rest only exist inside the image: ROS packages under /opt/ros, and the Orbbec
# driver the Dockerfile builds from source.
MISSING=""
for pkg in ur_description ur_robot_driver ur_client_library realsense2_description orbbec_description; do
    [[ -d "$FP/share/$pkg" ]] && { touch "$IDX/$pkg"; continue; }
    MISSING="$MISSING $pkg"
done
if [[ -z "$MISSING" ]]; then
    echo "  container packages already present"
elif docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    for pkg in $MISSING; do
        if docker exec "$CONTAINER" test -d "/opt/ros/jazzy/share/$pkg"; then
            docker exec "$CONTAINER" tar cC /opt/ros/jazzy/share -f - "$pkg" | tar xf - -C "$FP/share"
        else
            docker exec "$CONTAINER" tar cC "/home/ros/ros2_utils_ws/install/$pkg/share" -f - "$pkg" | tar xf - -C "$FP/share"
        fi
        touch "$IDX/$pkg"
    done
    echo "  copied$MISSING out of container '$CONTAINER'"
else
    echo "  MISSING$MISSING: start the container, then run this script again"
fi

say "URDF: $WS/mantis.urdf"
ROS_SETUP="${ROS_SETUP:-$HOME/rabota/SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash}"
if [[ -f "$WS/mantis.urdf" ]]; then
    echo "  already generated, left as is (delete it to regenerate)"
elif [[ ! -d "$FP/share/ur_description" || ! -d "$FP/share/orbbec_description" ]]; then
    echo "  skipped: the container packages are not in the prefix yet"
elif [[ ! -f "$ROS_SETUP" ]]; then
    echo "  skipped: host ROS env not found at $ROS_SETUP (set ROS_SETUP)"
else
    set +u
    source "$ROS_SETUP" 2>/dev/null
    set -u
    export AMENT_PREFIX_PATH="$FP:${AMENT_PREFIX_PATH:-}"
    python -c "import xacro; open('/tmp/mantis_gen.urdf','w').write(xacro.process_file('$WS/src/prl_ur5_ros2/prl_ur5_description/urdf/mantis.urdf.xacro', mappings={'gz_sim':'true'}).toprettyxml(indent='  '))"
    # Mesh paths come out pointing into the host prefix; the teleop resolves package://
    # against AMENT_PREFIX_PATH, so it works on the host and inside the container alike.
    sed -e "s|file://$FP/share/|package://|g" /tmp/mantis_gen.urdf > "$WS/mantis.urdf"
    echo "  generated ($(wc -l < "$WS/mantis.urdf") lines)"
fi

say "done"
echo "next:"
echo "  1. build the image and enter the container:"
echo "       cd <this repo>/../docker-ros2 && ./start_docker.bash $CONTAINER $SHARE"
echo "  2. inside it:  cd ~/share/mantis_ws && colcon build --symlink-install"
echo "                 pip install -e ~/share/lerobot_robot_mantis"
echo "  3. on the host: $SHARE/run_quest_pub.sh --scan"

#!/usr/bin/env bash
# Replay a recorded LeRobot episode on the Mantis via lerobot-replay.
set -eo pipefail
EP="${1:?usage: run_replay.sh <episode index> [task name] [extra args]}"
shift || true
NAME="${RECORD_NAME:-}"
if [[ $# -ge 1 && "$1" != --* ]]; then
    NAME="$1"
    shift
fi
SHARE="${SHARE_DIR:-/home/ros/share}"
ROOT="$SHARE/lerobot_data"
REPO=mantis/teleop
if [[ -n "$NAME" ]]; then
    ROOT="$ROOT/$NAME"
    REPO="mantis/$NAME"
fi

source /opt/ros/jazzy/setup.bash
source "$SHARE/mantis_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1

exec "$HOME/.local/bin/lerobot-replay" \
    --robot.type=mantis_follower \
    --robot.teleop_config="$SHARE/config_teleop_mantis.yaml" \
    --dataset.repo_id="$REPO" \
    --dataset.root="$ROOT" \
    --dataset.episode="$EP" \
    --play_sounds=false \
    "$@"

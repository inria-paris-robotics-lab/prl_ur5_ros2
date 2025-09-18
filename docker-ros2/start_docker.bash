#!/bin/bash

IMAGE_NAME="prl_ros2"
IMAGE_TAG=$(id -un)

xhost +local:docker > /dev/null 2>&1

# Check arguments
if [ "$#" -lt 2 ] || [ "$#" -gt 4 ]; then
  echo "Usage: $0 <container_name> <share directory path> <user(optional)> [--no-gpu] [--rebuild]"
  exit 1
fi

container_name=$1
storage=$2
user="ros"
gpu_enabled=true

# Shift out container_name and storage so we can parse remaining args
shift 2

for arg in "$@"; do
  if [ "$arg" = "--no-gpu" ]; then
    gpu_enabled=false
  elif [ "$arg" = "--rebuild" ]; then
    rebuild=true
  else
    user="$arg"
  fi
done

# Build the image if it doesn't exist
if ! docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" > /dev/null 2>&1 || [ "$rebuild" = true ]; then
  echo "Building Docker image ${IMAGE_NAME}:${IMAGE_TAG}..."
  docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" . --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) --build-arg DOCKER_GRP=$(getent group docker | cut -d: -f3) --build-arg INPUT_GRP=$(getent group input | cut -d: -f3)
fi


# Set working directory
if [ "$user" = "root" ]; then
  workdir="/root"
elif [ "$user" = "ros" ]; then
  workdir="/home/ros"
else
  echo "Invalid user: $user"
  exit 1
fi

echo "Launching your container..."

docker_cmd=(
  docker run -it --rm
  --env="DISPLAY"
  --env="QT_X11_NO_MITSHM=1"
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
  --volume="$XAUTHORITY:/dot.Xauthority"
  --privileged
  --net="host"
  -v /var/run/docker.sock:/var/run/docker.sock
  --name="$container_name"
  --user="$user"
  --workdir="$workdir"
  --mount type=bind,source="$storage",target="$workdir/share"
  --device=/dev/bus/usb 
  --group-add $(getent group dialout | cut -d: -f3)
  --group-add $(getent group docker  | cut -d: -f3)
  --group-add $(getent group input   | cut -d: -f3)
  --group-add $(getent group video   | cut -d: -f3)
)

# Add GPU runtime if needed
if [ "$gpu_enabled" = true ]; then
  docker_cmd+=(--runtime=nvidia --gpus all)
fi

docker_cmd+=("${IMAGE_NAME}:${IMAGE_TAG}" bash)

# Run the container
"${docker_cmd[@]}"

xhost -local:docker > /dev/null 2>&1
echo "Container exited"

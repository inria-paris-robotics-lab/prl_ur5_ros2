#!/bin/bash
IMAGE_NAME="prl_ros2"
IMAGE_TAG="jazzy"
# Check if the image exists locally
if !(docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" > /dev/null 2>&1); then
  docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" . --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
fi

xhost +local:docker > /dev/null 2>&1

# Check if the number of arguments is valid
if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "Usage: $0 <container_name> <share directory path> <user(optional)> "
  exit 1
fi
container_name=$1
storage=$2
user="${3:-ros}"
gpu_enabled=false

for arg in "$@"; do
  if [[ "$arg" == "--gpu" || "$arg" == "-g" ]]; then
    gpu_enabled=true
    break
  fi
done


# Set the working directory based on the user
if [ "$user" = "root" ]; then
  workdir="/root"
elif [ "$user" = "ros" ]; then
  workdir="/home/ros"
else
  echo "Invalid user"
  exit 1
fi

echo "Launching your container..."

if gpu_enabled; then
  docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTHORITY:/dot.Xauthority" \
  --privileged \
  --net="host" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  --name="$container_name" \
  --user="$user" \
  --workdir="$workdir" \
  --mount type=bind,source=$storage,target=$workdir/share \
  --runtime=nvidia \
  --gpu all \
  "${IMAGE_NAME}:${IMAGE_TAG}" \
  bash
else
  docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTHORITY:/dot.Xauthority" \
  --privileged \
  --net="host" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  --name="$container_name" \
  --user="$user" \
  --workdir="$workdir" \
  --mount type=bind,source=$storage,target=$workdir/share \
  "${IMAGE_NAME}:${IMAGE_TAG}" \
  bash
fi

xhost -local:docker > /dev/null 2>&1

echo "Container exited"

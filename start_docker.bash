xhost +local:docker > /dev/null 2>&1

# Check if the number of arguments is valid
if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "Usage: $0 <container_name> <docker volume name> <user(optional)> "
  exit 1
fi
container_name=$1
storage=$2
user="${3:-ros}"


# Set the working directory based on the user
if [ "$user" == "root" ]; then
  workdir="/root"
elif [ "$user" == "ros" ]; then
  workdir="/home/ros"
else
  echo "Invalid user"
  exit 1
fi

echo "Launching your container..."

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
--volume $storage:"$workdir/share" \
ros2 \
bash

xhost -local:docker > /dev/null 2>&1

echo "Container exited"


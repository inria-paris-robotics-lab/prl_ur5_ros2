
# Docker ROS 2 & UR Driver

This Dockerfile sets up a ROS 2 development environment with the ability to interface with UR-type cobots (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30) using the `ur_driver`.

## Features

Ubuntu 22.04 preinstalled with ROS 2 (humble), these versions are fully replaceable if you require others.\
UR Driver (`ur_driver`) for communication with Universal Robots.\
Ready-to-use environment to test and develop ROS 2 applications with UR cobots or others.

## Prerequisites

Before starting, make sure you have Docker installed on your machine.\
You can download and install Docker from the [official website](https://docs.docker.com/engine/install/).

## Installation

#### 1) Clone this repository
Clone this Git repository to your local machine:
```bash
$ git clone 
```

#### 2) Build the Docker image

Build the Docker image by running the following command in the project directory:
```bash
$ docker build -t ros2 .
```
This will download the base ROS 2 image and install all the necessary dependencies, tools, and the ROS 2 Universal Robot driver.

#### 3) Run the Docker container

Once the image is built, you can run a container from it using the provided shell script:

Before you start the container, create a volume with Docker to hold the files used in the container.
```bash
$ docker volume create <volume_name>
```
Anything created in the user's folder `share` will be stored on this volume.

If you want to retrieve your files or add some to a container, use the following command
```bash
$ docker cp <SRC> <DEST>
```

You can now start your container:

```bash
$ ./start_docker.bash <container_name> <user(optional)>
```
The container name is flexible and allows you to run multiple containers with the same image simultaneously.
There are by default two users: **root** and **ros**. It is recommended to use the **ros** user (non-root) to avoid creating root-owned files in the directory linked to the container on your machine.\
**ros** is the default user.

## Usage

Once inside the Docker container, you can use ROS 2 to interact with your UR robot. Here are some useful commands:

#### Run a UR simulation:
```bash
$ ros2 run ur_client_library start_ursim.sh -m <ur_type>
```
You can access the GUI of the emulated UR robot via the following link: http://192.168.56.101:6080/vnc.html 

If you are having difficulty accessing the port provided by the emulation node, please check your firewall and enable the port:
```bash
$ sudo ufw allow <port>/tcp
$ sudo ufw reload 
```

#### Run the driver for a physical robot:

Make sure your UR robot is connected to your network and that you can communicate with it using its IP.

Launch the UR driver in your ROS 2 environment with the following command:
```bash
$ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
```
Allowed `ur_type` strings: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30.

For more details, follow the official [UR driver guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html).

###
You can add any desired development tools by modifying the `# Dev image with tools and non-root user` section of the Dockerfile.
###

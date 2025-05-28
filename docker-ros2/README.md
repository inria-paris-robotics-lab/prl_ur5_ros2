
# Docker ROS 2 & UR Driver

This Dockerfile sets up a ROS 2 development environment with the ability to interface with UR-type cobots (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30) using the `ur_driver`.

## Prerequisites

#### Tested on linux/amd64, but not supported on ARM

Before starting, make sure you have Docker installed on your machine.\
You can download and install Docker from the [official website](https://docs.docker.com/engine/install/).

> **Note:** This setup uses the NVIDIA Container Toolkit to leverage your local GPU if you have an NVIDIA GPU. Please follow the [official NVIDIA Container Toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to install and configure the toolkit on your system.

## Installation

#### 1) Clone this repository
Clone this Git repository to your local machine:
```bash
git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git
```

#### 2) Set Up Your Environment 

> **Warning:** To ensure that your files are preserved inside the Docker container, a local folder is mounted into the container. Please create a dedicated folder on your host machine for this purpose before proceeding.

For example, you can create a folder named `docker_shared` in your home directory:
```bash
mkdir ~/docker_shared
```

#### 3) Build and Run the Docker Image

To build and run the Docker container, use the provided script. Make sure to specify the absolute path of the shared folder you created earlier:

```bash
./start_docker.bash <container_name> </absolute/path/to/shared_dir> <user(optional)>
```

- Replace `<container_name>` with a name of your choice for the container.
- Replace `</absolute/path/to/shared_dir>` with the absolute path to the shared folder on your host machine.
- Optionally, specify `<user>` as `ros` (default) or `root`.

> **Warning:** If you want to enable GPU sharing, add the `--gpu` option when running the `./start_docker.bash` script.

For example:
```bash
./start_docker.bash my_container ~/docker_shared ros
```

This command will start the Docker container with the specified configuration.
The container name is flexible and allows you to run multiple containers with the same image simultaneously.
There are by default two users: **root** and **ros**. It is recommended to use the **ros** user (non-root) to avoid creating root-owned files in the directory linked to the container on your machine.\
**ros** is the default user.

> **Warning:** Any files created outside the user's `shared` folder will be deleted after exiting the Docker container and permanently lost. Ensure all important files are saved within the `shared` folder to avoid data loss.

## Additional Notes on Usage 
Once inside the Docker container, you can use ROS 2 to interact with your UR robot. Here are some useful commands:

#### Run a UR simulation:
```bash
ros2 run ur_client_library start_ursim.sh -m <ur_type>
```
You can access the GUI of the emulated UR robot via the following link: http://192.168.56.101:6080/vnc.html

If you are having difficulty accessing the port provided by the emulation node, please check your firewall and enable the port:
```bash
sudo ufw allow <port>/tcp
sudo ufw reload
```

#### Run the driver for a physical robot:

Make sure your UR robot is connected to your network and that you can communicate with it using its IP.

Launch the UR driver in your ROS 2 environment with the following command:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
```
Allowed `ur_type` strings: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30.

For more details, follow the official [UR driver guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html).

###
You can add any desired development tools by modifying the `# Dev image with tools and non-root user` section of the Dockerfile.
###

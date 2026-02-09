
### **2. Install `prl` Packages**

Follow the steps below to set up the `prl` packages. These steps can be performed both inside and outside (only if you have ros2 jazzy locally) the Docker container. Ensure that the setup is done in the shared folder to maintain consistency and accessibility.

> [!IMPORTANT]
> Before proceeding with the setup, ensure you follow good practices for organizing your ROS 2 workspace. Create a folder to contain all your ROS 2 setup files. You can name it as you prefer, but in this guide, we will use `ws`. Inside this folder, create another folder named `src` to hold the source files.

To create these folders, use the following commands:
#### If you are using Docker

```bash
cd ~/share
mkdir -p ws/src
```

#### If you are working locally on your machine

```bash
mkdir -p ~/ws/src
```

This will create the `ws` directory in your home folder and the `src` directory inside it.

> [!WARNING]
> Remember that in the container, any changes made outside the `share` directory will not be saved after you shut down the container.

#### Clone the prl repository into your ROS 2 workspace:

```bash
cd ws/src
git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git
```

#### Install Dependencies

The **prl_ur5_description** package requires the following dependencies:

- [prl_ur5_robot_configuration](https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration)
- [universal_robot_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [universal_robot_gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2)
- [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [weiss_wsg50_ros](https://github.com/inria-paris-robotics-lab/wsg50-ros-pkg)
- [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)


These packages provide configuration files, robot descriptions, simulation models, and force-torque sensor drivers that are necessary for the UR5 robot to operate properly in a ROS2 ecosystem.

To install these dependencies, clone them into your workspace using the following commands:

```bash
cd ws/src
vcs import < prl_ur5_ros2/dependencies.repos 
```

#### Install Workspace dependencies

> [!IMPORTANT]
> To install dependencies and build the packages, you must have ROS 2 Jazzy installed locally. If you do not have ROS 2 Jazzy on your system, use the provided Docker environment (`docker-ros2`) for building and development.

After cloning the dependencies, check and install others dependencies linked to each packages with `rosdep`:

```bash
cd ..
sudo apt update
rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

# PRL UR5 Workbench

This project combines a ROS 2 development environment with Docker and packages for the description and simulation of the UR5 workbench from paris robotics lab.

## **Included Packages**

### 1. **docker-ros2**
This package provides a Docker environment for developing with ROS 2 (jazzy), including the **UR Driver** to interact with UR robots (UR3, UR5, UR10, etc.). The container is configured to work with these robots and includes all necessary tools for simulation and communication with both physical and simulated robots.

### 2. **prl_ur5_description**
The **prl_ur5_description** package provides the UR5 workbench description, including 3D models files necessary for visualizing and simulating the UR5 robot in a ROS 2 environment. It is part of the PRL (Paris Robotics Lab) ecosystem and designed to facilitate the use of the UR5 robot in simulation.

### 3. **prl_ur5_gazebo**
The **prl_ur5_gazebo** package provides the UR5 workbench launch and files, necessary for simulating the UR5 robot in Gazebo. It is part of the PRL (Paris Robotics Lab) ecosystem and designed to facilitate the use of the UR5 robot in simulation.

---

## **Prerequisites**

- Docker must be installed on your machine (Tested on `linux/amd64`, not supported on ARM).
- A compatible version of ROS 2 (Humble) must be installed and configured.
- Gazebo for simulating the UR5 robot (if you intend to use simulation).

---

Certainly! Below is the updated README where the installation sections for Docker and the `prl_ur5_description` package are separated for clarity.


## **Installation**

### **1. Docker Setup (for `docker-ros2`)**

**See [docker-ros2/README.md](docker-ros2/README.md)**

---

### **2. Install `prl_ur5_description` & `prl_ur5_gazebo` packages**

Follow the steps below to set up packages.

#### Clone the `prl_ur5_description` and `prl_ur5_gazebo`repository into your ROS 2 workspace:

```bash
cd ~/ws/src
git clone git@github.com:inria-paris-robotics-lab/prl_ur5_ros2.git
```

#### Install Dependencies

The **prl_ur5_description** package requires the following dependencies:

- [prl_ur5_robot_configuration](https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration)
- [universal_robot_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [universal_robot_gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2)
- [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

These packages provide configuration files, robot descriptions, simulation models, and force-torque sensor drivers that are necessary for the UR5 robot to operate properly in a ROS2 ecosystem.

To install these dependencies, clone them into your workspace using the following commands:

```bash
cd ~/ws/src
git clone -b ros2 https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
git clone -b ros2 https://github.com/inria-paris-robotics-lab/onrobot_ros.git
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
```

#### Install Workspace dependencies

After cloning the dependencies, check and install others dependencies linked to each packages with `rosdep`:

```bash
cd ~/ws
sudo apt update
rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

#### Build and source the Workspace

After you had istalled all dependencies you can build every packages with 'colcon':
```bash
colcon build --symlink-install --packages-skip onrobot_control onrobot_gazebo onrobot_ros robotiq_ft_sensor_hardware realsense2_camera
```

Once the build process is finished, source your workspace so that ROS 2 recognizes the new packages:

```bash
source install/setup.bash
```

---

## **Usage**

### Only visualize the workbench in RViz

```bash
ros2 launch prl_ur5_description view_workbench.launch.py
```

### Simulate the workbench in Gazebo and visualize in Rviz

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py
```

---

## **Important Notes**

- **ROS 2 Version**: Make sure you're using a compatible version of ROS 2. This guide assumes ROS 2 Jazzy.
- **Gazebo**: If you're using Gazebo for simulation, ensure it's installed and properly configured to work with ROS 2.
- **Dependency Issues**: If you face any issues with dependencies, refer to the individual documentation or open an issue in the relevant repository.



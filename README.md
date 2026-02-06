<table>
  <tr>
    <td><img src="doc/logo_mantis.png" alt="Logo" width="90"></td>
    <td><h1>PRL MANTIS</h1></td>
  </tr>
</table>

## **Project Overview**

This project integrates a ROS 2 development environment with Docker and provides packages for the description and simulation of the UR5 workbench, developed by the Paris Robotics Lab and referred to as **Mantis**.

<div align="center">
    <img src="doc/bimanual.png" alt="Bimanual UR" width="550">
</div>

## **Included Packages**

### 1. **docker-ros2**
This package provides a Docker environment for developing with ROS 2 (jazzy), including the **UR Driver** to interact with UR robots (UR3, UR5, UR10, etc.). The container is configured to work with these robots and includes all necessary tools for simulation and communication with both physical and simulated robots.

### 2. **prl_ur5_description**
The `prl_ur5_description` package provides the Mantis description, including 3D models files necessary for visualizing and simulating the UR5 robot in a ROS 2 environment.

### 3. **prl_ur5_gazebo**
The `prl_ur5_gazebo` package provides the Mantis launch and files, necessary for simulating the UR5 robot in Gazebo.

### 4. **prl_ur5_control**
The `prl_ur5_control` package provides configuration files for ROS 2 controllers and launch files to spawn the **Mantis** parts' different controllers.

### 5. **prl_ur5_moveit**
The `prl_ur5_moveit` package provides configuration and launch files to control the UR5 robot using various path planning solvers in ROS 2 with MoveIt.

### 6. **prl_ur5_run**
The `prl_ur5_run` package provides a launch file to access the real robot by starting the driver and enabling control.
---

## **Prerequisites**

- Docker must be installed on your machine (Tested on `linux/amd64`, not supported on ARM).
- A compatible version of ROS 2 (Humble) must be installed and configured.
- Gazebo for simulating the UR5 robot (if you intend to use simulation).

---

## **Installation**

### **1. Docker Setup (for `docker-ros2`)**

**See [docker-ros2/README.md](docker-ros2/README.md)**

### **2. Build and Run the Docker**

Create a shared folder on your host machine that will be mounted inside the Docker container. This folder will be used to store the Mantis workspace and any files you want to share between the host and the container.
```bash
mkdir ~/docker_shared
```

To build and run the Docker container, use the provided script. Make sure to specify the absolute path of the shared folder you created earlier:

```bash
cd prl_ur5_ros2/docker_ros2
./start_docker.bash my_container ~/docker_shared ros
```

> [!TIPS]
> If you want to disable GPU sharing, add the `--no-gpu` option when running the `./start_docker.bash` script.


> [!WARNING]
>  If you plan to use the Orbbec Femto Mega, you must install the Orbbec SDK ROS 2 on your local machine. Follow these steps:

1. Clone the Orbbec SDK ROS 2 repository:
  ```bash
  git clone https://github.com/orbbec/OrbbecSDK_ROS2.git -b v2-main
  ```

2. Install the udev rules:
  ```bash
  cd OrbbecSDK_ROS2/orbbec_camera/scripts
  sudo bash install_udev_rules.sh
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

Ensure these steps are completed before proceeding with the setup.

#### Rebuild and source the Workspace

After you had installed all dependencies you can rebuild every packages with 'colcon':

> [!NOTE]
> After the build, you may see an error related to the realsense package. You can ignore this error, as it does not affect the setup.


```bash
colcon build --symlink-install
```

Once the build process is finished, source your workspace so that ROS 2 recognizes the new packages:

```bash
source install/setup.bash
```

### **3. Setup Your Environment**

Before using Mantis, you need to make a few modifications to the configuration.
### **prl_ur5_robot_configuration**

To configure your setup, edit the `prl_ur5_robot_configuration/config/standard_setup.yaml` file. Update the following parameters to match your hardware and network setup:

- **IP Address and Ports**: Specify the network interface and ports for the robot.
- **Cameras**: Configure the hand-eye cameras, including their model and pose.
- **Gripper Type**: Define the type of gripper being used and its corresponding controller.
- **Fixed Camera**: Set up any fixed cameras required for your application.

Ensure all parameters are correctly adjusted to reflect your specific setup.

### **4. Usage Tips**

### **Use with Simulate Mantis**

> [!NOTE]
> The following instructions are simple examples. For the full list of launch arguments, refer to the README file in each respective package.

#### Only visualize Mantis in RViz

```bash
ros2 launch prl_ur5_description view_mantis.launch.py
```

#### Simulate Mantis in Gazebo and Visualize in RViz

To simulate Mantis in Gazebo and visualize it in RViz, use the following command:

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py
```

### **Using Simulation and MoveIt**

To use MoveIt with the Mantis, you can launch the simulation with the following command:

```bash
ros2 launch prl_ur5_run sim.launch.py
```

Alternatively, you can customize the launch by enabling or disabling specific components such as RViz, Gazebo GUI, or MoveIt. Use the following command with the desired parameters:

```bash
ros2 launch prl_ur5_run sim.launch.py launch_rviz:=<true|false> gazebo_gui:=<true|false> launch_moveit:=<true|false>
```

Replace `<true|false>` with `true` to enable or `false` to disable each component as needed.


### **Use with Real Robot**

To use the UR5 robot with a real setup, you need to modify the robot's network information in the standard setup file of the `prl_ur5_robot_configuration` package.

#### **Launch and Control the real Mantis**

Use the following command to launch control of the real robot with moveit:

```bash
ros2 launch prl_ur5_run real.launch.py
```
Alternatively, you can customize the launch by enabling or disabling specific components such as RViz or MoveIt. Use the following command with the desired parameters:
```bash
ros2 launch prl_ur5_run real.launch.py launch_rviz:=<true|false> launch_moveit:=<true|false>
```
Replace `<true|false>` with `true` to enable or `false` to disable each component as needed.

---

## **Important Notes**
For users intending to use the setup locally:
- **ROS 2 Version**: Ensure you are using a compatible version of ROS 2. This guide assumes ROS 2 Humble.
- **Gazebo**: Verify that Gazebo is installed and properly configured to work with ROS 2 for simulation purposes.

**Dependency Issues**: If you face any issues with dependencies, refer to the individual documentation or open an issue in the relevant repository.

# prl_ur5_description

## Description

The **prl_ur5_description** package provides the UR5 workbench robot description for use in a ROS2 environment. This package includes 3D models of the UR5 robot and necessary files to visualize and simulate the UR5 workbench in environments like Gazebo. 

This package is part of the PRL (Paris Robotics Lab) ecosystem and is intended to facilitate the use of the UR5 robot in simulation as well as provide integration with various robot components.

## Installation

Follow these steps to set up the package:

1. **Create a ROS2 workspace** (if you haven't done so already):

   ```bash
   mkdir -p ~/ws/src
   cd ~/ws/src
   ```

2. **Clone the repository** into your workspace:

   ```bash
   git clone <repository_link_here>
   ```

## Dependencies

The **prl_ur5_description** package requires the following dependencies:

- [prl_ur5_configuration](https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration)  
- [universal_robot_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)  
- [universal_robot_gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2)  
- [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver)  

These packages provide configuration files, robot descriptions, simulation models, and force-torque sensor drivers that are necessary for the UR5 robot to operate properly in a ROS2 ecosystem.

To install these dependencies, clone them into your workspace using the following commands:

```bash
cd ~/ws/src
git clone https://github.com/inria-paris-robotics-lab/prl_ur5_robot_configuration.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
```

## Build

Once the dependencies are installed, follow these steps to build the workspace:

1. **Initialize rosdep** if it hasn't been initialized already:

   ```bash
   rosdep init
   rosdep update
   ```

2. **Install the dependencies** using rosdep:

   ```bash
   rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```

3. **Build the workspace** using colcon:

   ```bash
   cd ~/ws
   colcon build --symlink-install
   ```

4. **Source your workspace**:

   ```bash
   source install/setup.bash
   ```

## Usage

To visualize the UR5 robot and its workbench in a simulation environment like Gazebo, you can launch the following command:

```bash
ros2 launch prl_ur5_description view_workbench.launch.py
```

This command will start the necessary nodes to bring up the robot in a simulation environment, allowing you to interact with the UR5 robot and its workbench. The visualization will be useful for testing, debugging, and developing your robot applications.

## Notes

- Ensure that you have a compatible ROS2 version installed and properly configured on your system.
- If you encounter issues with any of the dependencies, check their individual documentation or open an issue in the respective repositories.



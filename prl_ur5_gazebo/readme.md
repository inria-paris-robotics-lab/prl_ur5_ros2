# prl_ur5_gazebo

The `prl_ur5_gazebo` package provides the UR5 workbench launch and files, necessary for simulating the UR5 robot in Gazebo.

It is part of the PRL (Paris Robotics Lab) ecosystem and designed to facilitate the use of the UR5 robot in simulation.

## Content

### Launch Files
- **`start_gazebo_sim.launch.py`**  

Refer to the built-in documentation in the launch file header for more details.

### Configuration files
- **`gz_bridge.yaml`**
- **`camera_bridge.yaml`**

### Setup script
- **`generate_cameras_bridge.py`**

## Usage

### Configure Your Simulation Environment

Before launching the simulation, ensure your environment matches the camera configuration specified in the config files.

> **Note:** If you change the number of cameras or their active settings in your setup, you must update the camera bridge configuration file to reflect these changes.

To regenerate the camera bridge configuration file, run:

```bash
python3 <path_to_ws>/src/prl_ur5_ros2/prl_ur5_gazebo/scripts/generate_cameras_bridge.py -o <path_to_ws>/src/prl_ur5_ros2/prl_ur5_gazebo/config/camera_bridge.yaml
```

Replace `<path_to_ws>` with the path to your ROS 2 workspace.  
This will update `camera_bridge.yaml` to match your current camera setup.



### Launch Simulation

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py
```

#### Parameters:
- **`launch_rviz`**: Set to `true` to launch RViz alongside the simulation.
- **`gazebo_gui`**: Set to `true` to enable the Gazebo graphical interface.
- **`activate_cameras`**Set to `true` to activate the cameras in the simulation.**

Example usage:

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py launch_rviz:=true gazebo_gui:=true
```

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

## Usage

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py
```

#### Parameters:
- **`launch_rviz`**: Set to `true` to launch RViz alongside the simulation.
- **`gazebo_gui`**: Set to `true` to enable the Gazebo graphical interface.

Example usage:

```bash
ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py launch_rviz:=true gazebo_gui:=true
```

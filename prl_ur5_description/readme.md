# prl_ur5_description

The `prl_ur5_description` package provides the UR5 workbench description, including 3D model files necessary for visualizing and simulating the UR5 robot in a ROS 2 environment.

It is part of the PRL (Paris Robotics Lab) ecosystem and is designed to facilitate the use of the UR5 robot in simulation.

## Content

### Launch Files
- **`view_mantis.launch.py`**

Refer to the built-in documentation in the launch file header for more details.

### URDF Files
- **`workbench_ur5.urdf.xacro`**
- **`ur5_complete_arm.urdf.xacro`**
- **`_gripper.urdf.xacro`**
- **`_force_sensors.urdf.xacro`**
- **`_fixed_camera.urdf.xacro`**
- **`_camera_sensor.urdf.xacro`**
- **`d435_gazebo.urdf.xacro`**

### STL models
- **`vention_table.stl`**
- **`RG_connector_simple.stl`**
- **`RG_connector_simple_convex.stl`**


## Usage

```bash
ros2 launch prl_ur5_description view_mantis.launch.py
```

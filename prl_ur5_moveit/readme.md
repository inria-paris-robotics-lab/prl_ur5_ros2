# prl_ur5_moveit

The `prl_ur5_moveit` package provides configuration and launch files to control the UR5 robot using various path planning solvers in ROS 2 with MoveIt.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to simplify motion planning and execution for the UR5 robot in both simulation and real-world scenarios.

## Contents

### Launch Files
- **`start_moveit.launch.py`**  
    Launches the MoveIt configuration for the UR5 robot.
- **`setup_assistant.launch.py`**  
    Allows customization of the MoveIt package (e.g., solver, controller, etc.).

Refer to the documentation in the launch file headers for detailed usage instructions.

### Configuration Files
- **`moveit_controllers.yaml`**  
    Defines the controllers for the UR5 robot.
- **`kinematics.yaml`**  
    Contains kinematic parameters for motion planning.
- **`ur5.srdf`**  
    Describes the robot's semantic information.

## Usage

To launch the MoveIt configuration for the UR5 robot, use the following command:

```bash
ros2 launch prl_ur5_moveit start_moveit.launch.py
```

### Parameters
- **`use_sim_time`**: Indicates which clock MoveIt should use. Default: `true`.

> **Note**: Specify whether you are using a real robot or simulation by adding the appropriate parameter. By default, the MoveIt package is configured to work with simulation (`use_sim_time:=true`). For example, to use MoveIt with a real robot, set `use_sim_time` to `false`:

Example usage:

```bash
ros2 launch prl_ur5_moveit start_moveit.launch.py use_sim_time:=false
```
# prl_moveit_exemple

The `prl_moveit_exemple` package provides example files for interfacing your project with the PRL UR5 setup via MoveIt by using MoveItPy.

**`MoveItPy`** is a Python-based interface for MoveIt, enabling easier scripting and integration for robotic applications. It provides a high-level API to interact with MoveIt, making it simpler to plan and execute motions, configure robotic setups, and integrate custom behaviors. 

With `MoveItPy`, users can:
- Create and execute motion plans programmatically.
- Access and modify robot states and configurations.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to facilitate the use of the UR5 robot in simulation and real-world scenarios.

## Content

### Launch Files
- **`moveit_exemple.launch.py`**  
    This launch file demonstrates how to set up and run MoveIt configurations for the UR5 robot. Refer to the built-in documentation in the launch file headers for more details.

### Configuration Files
- **`planner.yaml`**  
    This file allows you to configure the planner for arm path planning using MoveIt. It includes parameters for optimizing motion planning and execution.

## Usage

To use this package, ensure that your environment is set up with the required dependencies, including MoveIt and MoveItPy. Follow these steps:

Launch the example using the provided launch file:
```bash
ros2 launch prl_moveit_exemple moveit_exemple.launch.py
```

Modify the `planner.yaml` file to customize the motion planning parameters as needed.

Refer to the official MoveItPy documentation and the executable built-in documentation for more advanced usage and integration tips.

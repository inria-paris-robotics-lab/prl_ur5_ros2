# prl_ur5_moveit

The `prl_ur5_moveit` package provides configuration and launch files to control the UR5 robot using various path planning solvers in ROS 2 with MoveIt.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to simplify motion planning and execution for the UR5 robot in both simulation and real-world scenarios.

## Contents

### Launch Files
- **`start_moveit.launch.py`**  
- **`setup_assistant.launch.py`**  


Refer to the documentation in the launch file headers for detailed usage instructions.

If you want to custom the moveit packagee (solver, controller, etc) use the `setup_assistant.launch.py`

### Configuration Files
- **`moveit_controllers.yaml`**  
- **`kinematics.yaml`**  
- **`ur5.srdf`**

## Usage

To launch the MoveIt configuration for the UR5 robot, use the following command:

```bash
ros2 launch prl_ur5_moveit start_moveit.launch.py use_sim_time:=<true|false>
```
Replace `<true|false>` with `true` if running in simulation, or `false` for real-world applications.
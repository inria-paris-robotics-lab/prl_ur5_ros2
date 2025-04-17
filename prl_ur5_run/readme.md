# prl_ur5_run

The `prl_ur5_run` package provides a launch file to access the real robot by starting the driver and enabling control.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to simplify motion planning and execution for the UR5 robot in both simulation and real-world scenarios.

## Contents

### Launch Files
- **`real.launch.py`**  

Refer to the documentation in the launch file headers for detailed usage instructions.

## Usage

To launch the MoveIt configuration for the UR5 robot, use the following command:

```bash
ros2 launch prl_ur5_run real.launch.py left_robot_ip:=<left_ip> right_robot_ip:=<right_ip>
```

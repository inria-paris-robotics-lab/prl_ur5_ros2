# prl_ur5_run

The `prl_ur5_run` package provides a launch file to access the real robot by starting the driver and enabling control.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to simplify motion planning and execution for the UR5 robot in both simulation and real-world scenarios.

## Contents

### Launch Files
- **`real.launch.py`**  
- **`sim.launch.py`**  

Refer to the documentation in the launch file headers for detailed usage instructions.

## Usage

To launch the MoveIt configuration for the Mantis, use the following command:

### Simulate :
```bash
ros2 launch prl_ur5_run sim.launch.py 
```

### Real :
```bash
ros2 launch prl_ur5_run real.launch.py left_robot_ip:=<left_ip> right_robot_ip:=<right_ip>
```
#### Parameters:
- **`left_robot_ip`**: IP address of the left robot. Default: `192.168.56.101`.
- **`right_robot_ip`**: IP address of the right robot. Default: `192.168.56.202`.
- **`left_kinematics_file`**: Path to the kinematics configuration file for the left robot. Default: `prl_ur5_robot_configuration/config/kinematics/ur5_left.yaml`.
- **`right_kinematics_file`**: Path to the kinematics configuration file for the right robot. Default: `prl_ur5_robot_configuration/config/kinematics/ur5_right.yaml`.
- **`headless_mode`**: Enable headless mode for robot control. Default: `true`.
- **`activate_joint_controller`**: Activate the default loaded joint controller. Default: `true`.
- **`activate_cameras`**: Activate cameras. Default: `false`.
- **`launch_rviz`**: Launch RViz. Default: `true`.
- **`launch_dashboard_client`**: Launch the Dashboard Client. Default: `false`.
- **`launch_urscript_interface`**: Launch the URScript Interface. Default: `false`.
- **`update_rate_config_file`**: Path to the update rate configuration file. Default: `ur_robot_driver/config/ur5_update_rate.yaml`.

Example usage:

```bash
ros2 launch prl_ur5_run real.launch.py \
    activate_cameras:=true \
    activate_joint_controller:=true \
    launch_rviz:=false
```

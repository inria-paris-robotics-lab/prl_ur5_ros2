# prl_ur5_control

The `prl_ur5_control` package provides configuration files for ROS 2 controllers and launch files to spawn mantis different controllers.

## Content

### Launch Files
- **`mantis_controllers.launch.py`**
- **`mantis_gripper_controllers.launch.py`**

Refer to the built-in documentation in the launch file headers for more details.

### Configuration Files
- **`dual_ur_controller.yaml`**
- **`left_ur_controller.yaml`**
- **`right_ur_controller.yaml`**
- **`wsg50_integrate.yaml`**

## Usage

These launch files are automatically launched by other packages, such as `prl_ur5_gazebo`. In practice, you typically do not need to launch them manually.

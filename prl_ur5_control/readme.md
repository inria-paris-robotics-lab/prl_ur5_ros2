# prl_ur5_control

The `prl_ur5_control` package provides configuration files for ROS 2 controllers and launch files to spawn the workbench parts different controllers.

It is part of the PRL (Paris Robotics Lab) ecosystem and is designed to facilitate the use of the UR5 robot in simulation.

## Content

### Launch Files
- **`workbench_controller.launch.py`**
- **`standard_setup_gripper_controller.launch.py`**

Refer to the built-in documentation in the launch file headers for more details.

### Configuration Files
- **`dual_arm_controller.yaml`**
- **`left_arm_controller.yaml`**
- **`right_arm_controller.yaml`**
- **`wsg50_integrate.yaml`**<!--  -->

## Usage

These launch files are automatically launched by other packages, such as `prl_ur5_gazebo`. In practice, you typically do not need to launch them manually.

# prl_ur5_control

The `prl_ur5_control` package provides configuration files for ROS 2 controllers and launch files to spawn the workbench parts different controllers.

It is part of the PRL (Paris Robotics Lab) ecosystem and is designed to facilitate the use of the UR5 robot in simulation.

## Content

### Launch Files
- **`mantis_controller.launch.py`**
- **`mantis_gripper_controller.launch.py`**
- **`sensors.launch.py`**

Refer to the built-in documentation in the launch file headers for more details.

> **Note:** The arguments for selecting camera parameters and camera type are stored in the `camera_config.yaml` file. Ensure that the camera type matches your hardware, and only use parameters supported by the corresponding launch file.

**Supported cameras:**
- **`realsense`**: Intel RealSense cameras
- **`femto-mega`**: Orbbec Femto Mega cameras

**Common arguments:**
- `camera_name`: Name of the camera.
- `type`: Camera type (`realsense` or `femto-mega`).

**Arguments for RealSense cameras:**
- `serial_no`: Serial number of the camera.
- `enable_depth`: Enable depth stream.
- `pointcloud.enable`: Enable point cloud generation.
- `enable_infra`: Enable infrared stream.
- `enable_infra1`: Enable infrared stream 1.
- `enable_infra2`: Enable infrared stream 2.
- `enable_color`: Enable color stream.
- `enable_gyro`: Enable gyroscope.
- `enable_accel`: Enable accelerometer.
- `enable_rgbd`: Enable RGB-D stream.
- `enable_sync`: Enable stream synchronization.
- `align_depth.enable`: Enable depth alignment.

**Arguments for Femto Mega cameras:**
- `net_device_ip`: IP address of the camera.
- `net_device_port`: Port number for network communication.
- `sync_mode`: Synchronization mode.
- `enable_colored_point_cloud`: Enable colored point cloud generation (`true` or `false`).
- `depth_registration`: Enable depth registration (`true` or `false`).

These arguments can be set the `camera_config.yaml` file to customize camera behavior.
If you need to add more arguments, you must also modify the launch files accordingly.


### Configuration Files
- **`dual_arm_controller.yaml`**
- **`left_arm_controller.yaml`**
- **`right_arm_controller.yaml`**
- **`wsg50_integrate.yaml`**<!--  -->

## Usage

These launch files are automatically launched by other packages, such as `prl_ur5_gazebo`. In practice, you typically do not need to launch them manually.

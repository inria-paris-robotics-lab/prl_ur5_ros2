from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def launch_setup(context):
    # Chemin absolu vers le fichier de configuration YAML
    config_file = Path(get_package_share_directory('prl_ur5_robot_configuration')) / 'config/fixed_cameras/cameras_config.yaml'
    
    # Chargement du fichier YAML
    with config_file.open('r') as setup_file:
        config = yaml.safe_load(setup_file)

    cameras = config.get('cameras', {})
    camera_launches = []

    nb_orbbec_activate = 0
    for camera_name, camera_info in cameras.items():
        if camera_info.get('type') == 'orbbec':
            # Count how many Orbbec cameras are activated
            if camera_info.get('activate', False):
                nb_orbbec_activate += 1

    for camera_name, camera_info in cameras.items():
        if camera_info.get('type') == 'realsense':
            activate = camera_info.get('activate', False)
            print(f"Camera {camera_name} activation status: {activate}")
            serial_no = camera_info.get('serial_no')
            enable_depth = str(camera_info.get('enable_depth', False)).lower()
            pointcloud = str(camera_info.get('pointcloud', False)).lower()
            enable_infra = str(camera_info.get('enable_infra', False)).lower()
            enable_infra1 = str(camera_info.get('enable_infra1', False)).lower()
            enable_infra2 = str(camera_info.get('enable_infra2', False)).lower()
            enable_color = str(camera_info.get('enable_color', False)).lower()
            enable_gyro = str(camera_info.get('enable_gyro', False)).lower()
            enable_accel = str(camera_info.get('enable_accel', False)).lower()
            enable_rgbd = str(camera_info.get('enable_rgbd', False)).lower()
            enable_sync = str(camera_info.get('enable_sync', False)).lower()
            enable_align_depth = str(camera_info.get('align_depth', False)).lower()
            if activate and serial_no:
                print(f"Activating camera {camera_name} with serial number {serial_no}")
                camera_node = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"
                    ]),
                    launch_arguments={
                        "camera_name": camera_name,
                        "serial_no": serial_no,
                        "enable_color": enable_color,
                        "enable_depth": enable_depth,
                        "pointcloud.enable": pointcloud,
                        "enable_infra": enable_infra,
                        "enable_infra1": enable_infra1,
                        "enable_infra2": enable_infra2,
                        "enable_gyro": enable_gyro,
                        "enable_accel": enable_accel,
                        "enable_sync": enable_sync,
                        "align_depth.enable": enable_align_depth,
                        "enable_rgbd" : enable_rgbd,

                    }.items()
                )
                camera_launches.append(camera_node)
        if camera_info.get('type') == 'femto-mega':
            activate = camera_info.get('activate', False)
            if activate:
                ip = camera_info.get('ip')
                port = camera_info.get('port')
                sync_mode = camera_info.get('sync_mode', 'standalone')
                enable_colored_point_cloud = str(camera_info.get('enable_colored_point_cloud', False))
                depth_registration = str(camera_info.get('depth_registration', False))
                camera_node = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("orbbec_camera"), "/launch", "/femto_mega.launch.py"
                    ]),
                    launch_arguments={
                        "camera_name": camera_name,
                        "net_device_ip": str(ip),
                        "net_device_port": str(port),
                        "sync_mode": sync_mode,
                        "enable_colored_point_cloud": enable_colored_point_cloud,
                        "depth_registration": depth_registration,
                    }.items(),
                )

                camera_launch_group = GroupAction(
                    actions=[
                        PushRosNamespace('camera'),
                        camera_node
                    ]
                )

                camera_launches.append(camera_launch_group)



    return camera_launches

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

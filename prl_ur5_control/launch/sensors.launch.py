from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetParameter

def launch_setup(context):
    config_file = Path(get_package_share_directory('prl_ur5_robot_configuration')) / 'config/fixed_cameras/cameras_config.yaml'
    
    with config_file.open('r') as setup_file:
        config = yaml.safe_load(setup_file)

    cameras = config.get('cameras', {})
    camera_launches = []
    
    # Ordering cameras by type and sync mode
    primary_cameras = []
    secondary_cameras = []
    realsense_cameras = []

    for camera_name, camera_info in cameras.items():
        if not camera_info.get('activate', False):
            continue
            
        if camera_info.get('type') == 'realsense':
            realsense_cameras.append((camera_name, camera_info))
        elif camera_info.get('type') == 'femto-mega':
            if camera_info.get('sync_mode') == 'primary':
                primary_cameras.append((camera_name, camera_info))
            else:
                secondary_cameras.append((camera_name, camera_info))

    def create_orbbec_node(camera_name, camera_info):
        sync_mode = camera_info.get('sync_mode', 'standalone')
        trigger_out = "true" if sync_mode == "primary" else "false"
        
        camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("orbbec_camera"), "/launch", "/femto_mega.launch.py"
            ]),
            launch_arguments={
                # --- CAMERA IDENTIFICATION ---
                "camera_name": camera_name,
                "net_device_ip": str(camera_info.get('ip')),
                "net_device_port": str(camera_info.get('port')),
                "enumerate_net_device": "false",
                
                # --- HARDWARE SYNC ---
                "sync_mode": sync_mode,
                "enable_frame_sync": str(camera_info.get('enable_frame_sync', True)).lower(),
                "depth_delay_us": str(camera_info.get('depth_delay_us', 0)),
                "color_delay_us": str(camera_info.get('color_delay_us', 0)),
                "trigger2image_delay_us": str(camera_info.get('trigger2image_delay_us', 0)),
                "trigger_out_delay_us": str(camera_info.get('trigger_out_delay_us', 0)),
                "trigger_out_enabled": trigger_out,
                
                # --- SOFTWARE SYNC---
                "time_domain": str(camera_info.get('time_domain', "system")),
                
                # --- FLUX AND RESOLUTION ---
                "enable_color": str(camera_info.get('enable_color', True)).lower(),
                "enable_depth": str(camera_info.get('enable_depth', True)).lower(),
                "enable_ir": str(camera_info.get('enable_ir', False)).lower(),
                
                "color_fps": str(camera_info.get('color_fps', 15)),
                "depth_fps": str(camera_info.get('depth_fps', 15)),
                "ir_fps": str(camera_info.get('ir_fps', 15)),
                
                "enable_point_cloud": str(camera_info.get('enable_point_cloud', True)).lower(),
                "enable_colored_point_cloud": str(camera_info.get('enable_colored_point_cloud', False)).lower(),
                "depth_registration": str(camera_info.get('depth_registration', False)).lower(),
                
                "publish_tf": "false",
                
                # --- FILTERING ---
                "enable_decimation_filter": str(camera_info.get('enable_decimation_filter', False)).lower(),
                "enable_spatial_filter": str(camera_info.get('enable_spatial_filter', False)).lower(),
                "spatial_filter_alpha": str(camera_info.get('spatial_filter_alpha', -1.0)),
                "spatial_filter_magnitude": str(camera_info.get('spatial_filter_magnitude', -1)),
                "spatial_filter_radius": str(camera_info.get('spatial_filter_radius', -1)),
                
                "enable_temporal_filter": str(camera_info.get('enable_temporal_filter', False)).lower(),
                "temporal_filter_weight": str(camera_info.get('temporal_filter_weight', -1.0)), 
                "temporal_filter_diff_threshold": str(camera_info.get('temporal_filter_diff_threshold', -1.0)), 
                
                "enable_hole_filling_filter": str(camera_info.get('enable_hole_filling_filter', False)).lower(),
                "hole_filling_filter_mode": str(camera_info.get('hole_filling_filter_mode', "")),
                
                "enable_threshold_filter": str(camera_info.get('enable_threshold_filter', False)).lower(),
                "threshold_filter_max": str(camera_info.get('threshold_filter_max', -1)),
                "threshold_filter_min": str(camera_info.get('threshold_filter_min', -1)),
                
                "enable_noise_removal_filter": str(camera_info.get('enable_noise_removal_filter', False)).lower(),
                "noise_removal_filter_min_diff": str(camera_info.get('noise_removal_filter_min_diff', 10)),
                "noise_removal_filter_max_size": str(camera_info.get('noise_removal_filter_max_size', 50)),
                
            }.items(),
        )
        
        return GroupAction(
            actions=[
                PushRosNamespace('camera'),
                SetParameter(name='enable_sync_host_time', value=False), 
                camera_node
            ]
        )

    for camera_name, camera_info in realsense_cameras:
        serial_no = camera_info.get('serial_no')
        if serial_no:
            camera_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"
                ]),
                launch_arguments={
                    "camera_name": camera_name,
                    "serial_no": serial_no,
                    "enable_color": str(camera_info.get('enable_color', False)).lower(),
                    "enable_depth": str(camera_info.get('enable_depth', False)).lower(),
                    "pointcloud.enable": str(camera_info.get('pointcloud', False)).lower(),
                    "enable_infra": str(camera_info.get('enable_infra', False)).lower(),
                    "enable_infra1": str(camera_info.get('enable_infra1', False)).lower(),
                    "enable_infra2": str(camera_info.get('enable_infra2', False)).lower(),
                    "enable_gyro": str(camera_info.get('enable_gyro', False)).lower(),
                    "enable_accel": str(camera_info.get('enable_accel', False)).lower(),
                    "enable_sync": str(camera_info.get('enable_sync', False)).lower(),
                    "align_depth.enable": str(camera_info.get('align_depth', False)).lower(),
                    "enable_rgbd": str(camera_info.get('enable_rgbd', False)).lower(),
                }.items()
            )
            camera_launches.append(camera_node)

    delay = 0.0 # ORRBEC must be launched with delay between each one 
    for camera_name, camera_info in secondary_cameras:
        orbbec_node = create_orbbec_node(camera_name, camera_info)
        
        camera_launches.append(
            TimerAction(
                period=delay,
                actions=[orbbec_node]
            )
        )
        delay += 2.0

    # Primary camera need to be launched as last orbbec 
    for camera_name, camera_info in primary_cameras:
        orbbec_node = create_orbbec_node(camera_name, camera_info)
        
        camera_launches.append(
            TimerAction(
                period=delay,
                actions=[orbbec_node]
            )
        )
        delay += 2.0

    return camera_launches

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

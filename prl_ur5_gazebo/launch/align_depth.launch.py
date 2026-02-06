import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from pathlib import Path

def launch_setup(context):
    config_file = Path(LaunchConfiguration('camera_config').perform(context))

    with config_file.open('r') as setup_file:
        config = yaml.safe_load(setup_file)

    cameras = config.get('cameras', {})
    camera_align = []
    for camera_name, camera_info in cameras.items():
        activate = camera_info.get('activate', False)
        if activate:
            enable_align_depth = camera_info.get('align_depth', False)
            print(f"Camera {camera_name} alignment enabled: {enable_align_depth}")
            if enable_align_depth:
                ros_color_info_topic = f'camera/{camera_name}/color/camera_info'
                ros_depth_info_topic = f'camera/{camera_name}/depth/camera_info'
                ros_depth_image_topic = f'camera/{camera_name}/depth/depth_image'
                output_topic = f'camera/{camera_name}/aligned_depth_to_color/image_raw'
                camera_align.append(
                    Node(
                        package='depth_image_proc',
                        executable='register_node',
                        name=f'depth_image_proc_register_{camera_name}',
                        parameters=[{'use_sim_time': True}],
                        remappings=[
                            ('rgb/camera_info', ros_color_info_topic),
                            ('depth/camera_info', ros_depth_info_topic),
                            ('depth/image_rect', ros_depth_image_topic),
                            ('depth_registered/image_rect', output_topic)
                        ]
                    )
                )

    return camera_align

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('prl_ur5_robot_configuration'),
                'config/fixed_cameras',
                'cameras_config.yaml'
            ]),
            description='Path to the camera configuration file.'
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

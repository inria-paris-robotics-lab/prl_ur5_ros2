from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def launch_setup(context):
    # Arguments 
    enable_alpha = LaunchConfiguration("enable_alpha")
    alpha_id = LaunchConfiguration("alpha_id")
    enable_bravo = LaunchConfiguration("enable_bravo")
    bravo_id = LaunchConfiguration("bravo_id")
    enable_charlie = LaunchConfiguration("enable_charlie")
    charlie_id = LaunchConfiguration("charlie_id")
    enable_delta = LaunchConfiguration("enable_delta")
    delta_id = LaunchConfiguration("delta_id")


    alpha_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch","/rs_launch.py"]
        ),
        launch_arguments={
            "camera_name": "alpha_camera",
            "serial_no":str(alpha_id),
            "enable_depth": "true",
        }.items(),
        condition=IfCondition(enable_alpha),
    )

    bravo_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch","/rs_launch.py"]
        ),
        launch_arguments={
            "camera_name": "bravo_camera",
            "serial_no":str(bravo_id),
            "enable_depth": "true",
        }.items(),
        condition=IfCondition(enable_bravo),
    )
    
    charlie_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch","/rs_launch.py"]
        ),
        launch_arguments={
            "camera_name": "charlie_camera",
            "serial_no":str(charlie_id),
            "enable_depth": "true",
        }.items(),
        condition=IfCondition(enable_charlie),
    )

    delta_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch","/rs_launch.py"]
        ),
        launch_arguments={
            "camera_name": "delta_camera",
            "serial_no":str(delta_id),
            "enable_depth": "true",
        }.items(),
        condition=IfCondition(enable_delta),
    )

    return [
        alpha_camera,
        bravo_camera,
        charlie_camera,
        delta_camera,
    ]




def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "enable_alpha",
            default_value="true",
            description="Enable (or disable) alpha_camera",
        ),
        DeclareLaunchArgument(
            "alpha_id",
            default_value="0",
            description="Alpha camera serial number",
        ),
        DeclareLaunchArgument(
            "enable_bravo",
            default_value="true",
            description="Enable (or disable) bravo_camera",
        ),
        DeclareLaunchArgument(
            "bravo_id",
            default_value="0",
            description="Bravo camera serial number",
        ),
        DeclareLaunchArgument(
            "enable_charlie",
            default_value="true",
            description="Enable (or disable) charlie_camera",
        ),
        DeclareLaunchArgument(
            "charlie_id",
            default_value="0",
            description="Charlie camera serial number",
        ),
        DeclareLaunchArgument(
            "enable_delta",
            default_value="true",
            description="Enable (or disable) delta_camera",
        ),
        DeclareLaunchArgument(
            "delta_id",
            default_value="0",
            description="Delta camera serial number",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])




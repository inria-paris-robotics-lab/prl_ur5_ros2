from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    # Get the launch arguments
    launch_rviz = context.launch_configurations.get("launch_rviz", "false")
    gazebo_gui = context.launch_configurations.get("gazebo_gui", "true")
    launch_moveit = context.launch_configurations.get("launch_moveit", "true")
    activate_cameras = context.launch_configurations.get("activate_cameras", "false")


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("prl_ur5_gazebo"),
                "launch",
                "start_gazebo_sim.launch.py",
            ])
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "gazebo_gui": gazebo_gui,
            "activate_cameras": activate_cameras,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("prl_ur5_moveit"),
                "launch",
                "start_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(launch_moveit),
    )

    return [
        gazebo_launch,
        moveit_launch,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Launch Gazebo gui if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description="Launch MoveIt if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_cameras",
            default_value="false",
            description="Activate cameras if true",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

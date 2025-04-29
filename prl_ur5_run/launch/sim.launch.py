from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("prl_ur5_gazebo"),
                "launch",
                "start_gazebo_sim.launch.py",
            ])
        ),
        launch_arguments={
            "launch_rviz": "false"
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
    )

    return [
        gazebo_launch,
        moveit_launch,
    ]

def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
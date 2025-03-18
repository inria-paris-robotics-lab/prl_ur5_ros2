from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context):
    # Initialize Arguments
    
    controller = LaunchConfiguration("gripper_controller").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    print('controller:', controller)
    if controller == "weiss-gripper":
        controller_to_start =IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wsg_50_simulation'),
                    'launch',
                    'wsg50_controllers.launch.py',
                ])
            ]),
            launch_arguments=[
                ('prefix', prefix),
            ],
        )
    else:
        raise RuntimeError("Unknown gripper controller '{}'".format(controller))
    

    return [controller_to_start]


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_controller",
            description="Gripper controller to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            description="Robot prefix",
            default_value="",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
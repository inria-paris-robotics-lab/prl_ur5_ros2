############################################################################################################
# Description: This launch file is used to start the gripper controller based on the argument provided.
# Arguments:
#   - gripper_controller: Gripper controller to use.
#   - prefix: Robot prefix. Default is "".
# Usage:
#   - ros2 launch prl_ur5_control mantis_gripper_controllers.launch.py gripper_controller:=<gripper-type> prefix:=<tf-prefix>
############################################################################################################
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    # Initialize Arguments
    controller = LaunchConfiguration("gripper_controller").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)

    # Start the controller based on the argument
    controllers_to_start = []
    if controller == "weiss-gripper":
        weiss_controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wsg_50_simulation'),
                    'launch',
                    'wsg_50_controllers.launch.py',
                ])
            ]),
            launch_arguments=[
                ('prefix', prefix),('controller_file', PathJoinSubstitution([FindPackageShare('prl_ur5_control'), 'config', 'wsg50_integrate.yaml']))
            ],
        )
        controllers_to_start.append(weiss_controller)
    elif not controller or controller.lower() == "none":
        pass
    else:
        raise RuntimeError("Unknown gripper controller '{}'".format(controller))
    return controllers_to_start


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
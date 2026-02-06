############################################################################################################
# Description: This launch file is used to load the controllers for the mantis robot.
# The controllers are loaded in the controller manager and can be activated or deactivated.
# Arguments:
# - activate_joint_controller: Activate wanted loaded joint controller.
# - initial_joint_controller: Initially activated robot controller (comma-separated). (used if activate_joint_controller is true)
# Usage:
# - ros2 launch prl_ur5_control mantis_controllers.launch.py
############################################################################################################
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def controller_spawner(controllers, active=True):
    inactive_flags = ["--inactive"] if not active else []
    spawners=[]
    for controller in controllers:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "10",
                ]
                + inactive_flags
                + [controller],
            )
        )
    return spawners

def launch_setup(context):
    # Arguments
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    active_controller = LaunchConfiguration("active_controller").perform(context)
    loaded_controllers = LaunchConfiguration("loaded_controllers").perform(context)

    # Convert string to list
    controllers_to_activate = active_controller.split(",")
    controllers_to_load = loaded_controllers.split(",")

    # Default active controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
            "joint_state_broadcaster",
        ],
    )
    controllers_active = []
    # Default inactive controllers
    controllers_inactive = []
    # Update active and inactive controllers with initial joint controller
    if activate_joint_controller.lower() == "true":
        for controller in controllers_to_activate:
            controllers_active.append(controller)
        for controller in controllers_to_load:
            if controller not in controllers_active :
                controllers_inactive.append(controller)

    controller_spawners = (
        controller_spawner(controllers_active)+
        controller_spawner(controllers_inactive, active=False)
    )

    return [
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=controller_spawners,
            )
        ),
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate wanted joint controller",
        ),
        DeclareLaunchArgument(
            "active_controller",
            default_value="left_joint_trajectory_controller,right_joint_trajectory_controller",
            description="Initially activated robot controller (comma-separated). (used if activate_joint_controller is true)",
        ),
        DeclareLaunchArgument(
            "loaded_controllers",
            default_value="",
            description="Just load the specified controllers (comma-separated).",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

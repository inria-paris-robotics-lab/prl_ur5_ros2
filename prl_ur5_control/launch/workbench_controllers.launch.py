from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def controller_spawner(controllers, active=True):
    inactive_flags = ["--inactive"] if not active else []
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
        ]
        + inactive_flags
        + controllers,
    )


def launch_setup(context):
    # Arguments 
    headless_mode = LaunchConfiguration("headless_mode").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)

    # Convert string to list
    controllers_to_activate = initial_joint_controller.split(",")

    # Default active controllers
    controllers_active = ["joint_state_broadcaster"]
    # Default inactive controllers
    controllers_inactive = [
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "left_force_mode_controller",
        "right_force_mode_controller",
        "left_passthrough_trajectory_controller",
        "right_passthrough_trajectory_controller",
        "left_freedrive_mode_controller",
        "right_freedrive_mode_controller",
        "left_io_and_status_controller",
        "right_io_and_status_controller",
        "left_speed_scaling_state_broadcaster",
        "right_speed_scaling_state_broadcaster",
        "left_force_torque_sensor_broadcaster",
        "right_force_torque_sensor_broadcaster",
        "left_tcp_pose_broadcaster",
        "right_tcp_pose_broadcaster",
        "left_ur_configuration_controller",
        "right_ur_configuration_controller",
        "left_scaled_joint_trajectory_controller",
        "right_scaled_joint_trajectory_controller",
    ]
    # Update active and inactive controllers with initial joint controller
    if activate_joint_controller.lower() == "true":
        for controller in controllers_to_activate:
            if controller in controllers_inactive:
                controllers_inactive.remove(controller)
            controllers_active.append(controller)

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    return controller_spawners


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="left_joint_trajectory_controller,right_joint_trajectory_controller",
            description="Initially loaded robot controller (comma-separated). The controllers need to be loaded in the controller manager. (used if activate_joint_controller is true)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

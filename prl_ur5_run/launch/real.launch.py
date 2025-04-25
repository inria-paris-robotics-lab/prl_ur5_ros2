############################################################################################################
# Description: This file is used to connect to the real robot with the ros2 environment.
#              The launch file starts the following nodes:
#               - Controller Manager
#               - Controller Spawners
#               - Dashboard Client
#               - Robot State Helper
#               - URScript Interface
#               - RViz
#               - Joint State Publisher
#              The launch file also includes the following launch files:
#               - mantis_controllers.launch.py
# Arguments:
#               - left_robot_ip: IP address of the left robot
#               - right_robot_ip: IP address of the right robot
#               - activate_joint_controller: Activate wanted joint controller.
#               - launch_rviz: Launch RViz
#               - launch_dashboard_client: Launch Dashboard Client
#               - launch_urscript_interface: Launch URScript Interface
#               - left_kinematics_file: Left robot kinematics file
#               - right_kinematics_file: Right robot kinematics file
#               - update_rate_config_file: Update rate configuration file
# Usage:
#               $ ros2 launch prl_ur5_control real.launch.py left_robot_ip:=<left_robot_ip> right_robot_ip:=<right_robot_ip>
############################################################################################################
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import yaml


def launch_setup(context):
    # Setup file
    controllers_file = PathJoinSubstitution([FindPackageShare("prl_ur5_control"), "config", "dual_ur_controller.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"])
    right_kinematics_file = LaunchConfiguration("right_kinematics_file")
    left_kinematics_file = LaunchConfiguration("left_kinematics_file")
    config_file = os.path.join(get_package_share_directory('prl_ur5_robot_configuration'), 'config', 'standart_setup.yaml')
    # Generals Arguments
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    launch_urscript_interface = LaunchConfiguration("launch_urscript_interface")
    activate_cameras = LaunchConfiguration("activate_cameras")
    launch_moveit = LaunchConfiguration("launch_moveit")

    ###### Calibration ######

    right_calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('ur_calibration'),
            'launch',
            'calibration_correction.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_ip': right_robot_ip,
            'target_filename': right_kinematics_file,
        }.items(),
    )

    left_calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('ur_calibration'),
            'launch',
            'calibration_correction.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_ip': left_robot_ip,
            'target_filename': left_kinematics_file,
        }.items(),
    )


    ###### Controllers ######

    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file),
        ],
        output="screen",
    )
    
    # Spawn controllers
    initial_joint_controller = ",".join([
        "left_io_and_status_controller",
        "right_io_and_status_controller",
        "left_speed_scaling_state_broadcaster",
        "right_speed_scaling_state_broadcaster",
        "left_force_torque_sensor_broadcaster",
        "right_force_torque_sensor_broadcaster",
        "left_tcp_pose_broadcaster",
        "right_tcp_pose_broadcaster",
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
    ])
    controller_spawners = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('prl_ur5_control'),
            'launch',
            'mantis_controllers.launch.py',
            ])
        ]),
        launch_arguments={
            'activate_joint_controller': activate_joint_controller,
            'initial_joint_controller': initial_joint_controller,
        }.items(),
    )

    ###### UR Driver Side ######

    # Dashboard client node, it enable us to made everything like we have the robot dashboard
    left_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(launch_dashboard_client),
        executable="dashboard_client",
        name="left_dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": left_robot_ip}],
    )

    right_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(launch_dashboard_client),
        executable="dashboard_client",
        name="right_dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": right_robot_ip}],
    )

    # The robot_state_helper node can be used to start the robot, release the brakes, and (re-)start the program through an action call.
    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": True},
        ],
    )

    # The URScript interface node is used to send URScript commands to the robot controller directly 
    left_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": left_robot_ip}],
        name="left_urscript_interface",
        output="screen",
        condition=IfCondition(launch_urscript_interface),
    )

    right_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": right_robot_ip}],
        name="right_urscript_interface",
        output="screen",
        condition=IfCondition(launch_urscript_interface),
    )

    ###### RViz ######
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ###### Joint state Publisher ######

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # Find the xacro executable
            " ", 
            PathJoinSubstitution([FindPackageShare("prl_ur5_description"), "urdf", "mantis.urdf.xacro"]),
            " ",
            "gz_sim:=",
            "false",
            " ",
            "real_robot:=",
            "true",
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ###### MoveIt ######
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("prl_ur5_moveit"),
                "launch",
                "start_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )


    return [
        right_calib,
        left_calib,
        control_node,
        controller_spawners,
        left_dashboard_client_node,
        right_dashboard_client_node,
        left_urscript_interface,
        right_urscript_interface,
        # robot_state_helper_node,
        rsp,
        rviz_node,
        moveit_launch,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_robot_ip",
            default_value="192.168.56.101",
            description="IP address of the left robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_robot_ip",
            default_value="192.168.56.202",
            description="IP address of the right robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_kinematics_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_ur5_robot_configuration"),
                        "config",
                        "kinematics",
                    ]
                ),
                "/ur5_left.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_kinematics_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_ur5_robot_configuration"),
                        "config",
                        "kinematics",
                    ]
                ),
                "/ur5_right.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_cameras",
            default_value="false",
            description="Activate cameras?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="false",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_urscript_interface",
            default_value="false",
            description="Launch URScript Interface?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "config",
                    ]
                ),
                "/ur5_update_rate.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_moveit",
            default_value="true",
            description="Launch MoveIt?",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

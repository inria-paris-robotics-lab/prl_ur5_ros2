############################################################################################################
# Description: This launch file starts the Gazebo simulation with the UR5 workbench.
#              The launch file starts the following nodes:
#              - Robot state publisher
#              - Gazebo
#              - Ignition spawn entity
#              - Gazebo Bridge
#              - Rviz
#              The launch file also includes the following launch files:            
#              - Gazebo simulation launch file
#              - Workbench controllers launch file
#              - Gripper controllers launch file
#              The launch file also includes the following configuration files:
#              - Workbench URDF description file that contains the robot description
#              - dual_arm_controller file that contains each UR controller configuration
#              - Bridge parameters file that contains the configuration of the bridge between ROS and Gazebo
#              - Rviz configuration file that contains the rviz setup
#              - World file that contains the workbench setup world
#              - Standart setup file that contains the configuration of the workbench setup
# Arguments:
#   - use_sim_time: Use simulation (Gazebo) clock if true
#   - launch_rviz: Launch rviz
#   - gazebo_gui: Launch gazebo with GUI
# Usage:
#   $ ros2 launch prl_ur5_gazebo start_gazebo_sim.launch.py
############################################################################################################
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
    # Launch Arguments 
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    launch_rviz = LaunchConfiguration('launch_rviz', default=True)
    gazebo_gui = LaunchConfiguration('gazebo_gui', default=True)
    # Load the configuration file
    config_file = os.path.join(get_package_share_directory('prl_ur5_robot_configuration'), 'config', 'standart_setup.yaml')
    description_file=PathJoinSubstitution([FindPackageShare('prl_ur5_description'),'urdf', 'mantis.urdf.xacro'])
    dual_controller_file=PathJoinSubstitution([FindPackageShare('prl_ur5_control'),'config', 'dual_arm_controller.yaml'])
    bridge_params = os.path.join(get_package_share_directory('prl_ur5_gazebo'), 'config', 'gz_bridge.yaml')
    rviz_config_file = PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'rviz', 'config.rviz'])
    world_file = PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'world', 'default.sdf'])
    camera_bridge_params = os.path.join(get_package_share_directory('prl_ur5_gazebo'), 'config', 'camera_bridge.yaml')

    ###### Robot description ######

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
            "gz_sim:=",
            "true",
            " ",
        ]
    )
    robot_description = {'robot_description':  ParameterValue(value=robot_description_content, value_type=str)}
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    ###### Rviz ######
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    ###### Gazebo ######

    # Gazebo launch 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )

    # Spawn entity in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'mantis',
                   '-allow_renaming', 'true'],
        parameters=[{"use_sim_time": True}],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    # Camera bridge
    camera_bridge= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={camera_bridge_params}',
        ],
        output='screen',
    )

    ###### Controllers ######

    # UR controllers
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('prl_ur5_control'),
            'launch',
            'mantis_controllers.launch.py',
            ])
        ]),
    )
    # Gripper controllers
    config_path = Path(config_file) 
    with config_path.open('r') as setup_file:
        config = yaml.safe_load(setup_file)

    left_gripper_controller = config.get('left')['gripper_controller']
    right_gripper_controller = config.get('right')['gripper_controller']

    left_gripper_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('prl_ur5_control'),
                'launch',
                'mantis_gripper_controllers.launch.py',
            ])
        ]),
        launch_arguments=[
            ('gripper_controller', left_gripper_controller),
            ('prefix', 'left_'),
        ],
    )
    right_gripper_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('prl_ur5_control'),
                'launch',
                'mantis_gripper_controllers.launch.py',
            ])
        ]),
        launch_arguments=[
            ('gripper_controller', right_gripper_controller),
            ('prefix', 'right_'),
        ],
    )
    return [robot_state_publisher, 
            gazebo,
            bridge,
            ignition_spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[
                        controller_launch,
                        left_gripper_controller,
                        right_gripper_controller,
                        # camera_bridge,
                            ],
                )
            ),
            rviz_node,
            ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
    )   
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Launch rviz'),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='True',
            description='Launch gazebo with GUI'),
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

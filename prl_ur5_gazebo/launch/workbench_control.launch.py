import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    description_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'urdf', 'workbench_gazebo.urdf.xacro'])
    
    dual_controller_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'config', 'dual_ur_controller.yaml'])
    
    output=PathJoinSubstitution("tmp.xacro")
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
        ]
    )

    # Wrap robot_description in ParameterValue to treat it as a string
    robot_description = {'robot_description':  ParameterValue(value=robot_description_content, value_type=str)}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'rviz', 'config.rviz'])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn entity in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'dual_ur',
                   '-allow_renaming', 'true'],
    )


    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            ],
    )
    ur_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "forward_velocity_controller",
            '--param-file', dual_controller_file,
            ],
    )

    
    # Bridge between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Gazebo world file
    gz_world = PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'world', 'default.sdf'])


    return LaunchDescription([
        LogInfo(msg="Startin gazebo launch..."),
        node_robot_state_publisher,
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                      'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ['-r -v 4 ', gz_world])],),
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[
                  LogInfo(msg="Spawning controllers"),
                  joint_state_broadcaster_spawner,
                  ur_base_controller_spawner,
                        ],
            )
        ),
        ignition_spawn_entity,
        rviz_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

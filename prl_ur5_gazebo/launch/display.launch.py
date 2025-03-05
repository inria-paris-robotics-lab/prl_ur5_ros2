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

    description_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'urdf', 'dual.urdf.xacro'])
    env_description_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'urdf', 'env.urdf.xacro'])
    
    left_controller_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'config', 'left_ur_controller.yaml'])
    right_controller_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'config', 'right_ur_controller.yaml'])
    
    output=PathJoinSubstitution("tmp.xacro")
    # Get URDF via xacro
    left_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
            "name:=left",
            " ",
            "prefix:=left_",
            " ",
            "workbench:=true",
            " ",
            "parent:=prl_ur5_base",
        ]
    )

    right_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
            "name:=right",
            " ",
            "prefix:=right_",
            " ",
            "parent:=prl_ur5_base",


        ]
    )





    # Wrap robot_description in ParameterValue to treat it as a string
    left_robot_description = {'robot_description':  ParameterValue(value=left_robot_description_content, value_type=str)}
    right_robot_description = {'robot_description':  ParameterValue(value=right_robot_description_content, value_type=str)}

    # Robot state publisher
    left_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='left_ur',
        output='both',
        parameters=[left_robot_description, {'use_sim_time': use_sim_time}],
    )
    right_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='right_ur',
        output='both',
        parameters=[right_robot_description, {'use_sim_time': use_sim_time}],
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
    ignition_spawn_entity_left = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='left_ur',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'left_ur',
                   '-allow_renaming', 'true'],
    )
    ignition_spawn_entity_right = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='right_ur',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'right_ur',
                   '-allow_renaming', 'true'],
    )





    # Controller spawners left

    left_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/left_ur/controller_manager'
            ],
    )
    left_ur_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "forward_velocity_controller",
            '--param-file', left_controller_file,
            '--controller-manager', '/left_ur/controller_manager'
            ],
    )

    # Controller spawners right

    right_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/right_ur/controller_manager'
            ],
    )
    right_ur_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "forward_velocity_controller",
            '--param-file', right_controller_file,
            '--controller-manager', '/right_ur/controller_manager'
            ],
    )

   
    
    

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    gz_world = PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'world', 'default.sdf'])

    return LaunchDescription([
        LogInfo(msg="Startin gazebo launch..."),
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])],),
        
        left_robot_state_publisher,
        right_robot_state_publisher,
        ignition_spawn_entity_left,
        ignition_spawn_entity_right,
        left_joint_state_broadcaster_spawner,
        right_joint_state_broadcaster_spawner,
        left_ur_base_controller_spawner,
        right_ur_base_controller_spawner,

        rviz_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

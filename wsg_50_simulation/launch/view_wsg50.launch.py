
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription ,RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('wsg_50_simulation'),
        'urdf',
        'wsg_50.urdf'
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            ""
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'wsg50',
                   '-allow_renaming', 'true'],
    )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                      'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])],)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
#       <!-- Launch the joint controller -->
#   <rosparam file="$(find wsg_50_simulation)/controllers/wsg_50_gr.yaml" command="load" />
#   <node pkg="pr2_controller_manager" type="spawner" args="wsg_50_gr" name="wsg_50_gr" />

#   <rosparam file="$(find wsg_50_simulation)/controllers/wsg_50_gl.yaml" command="load" />
#   <node pkg="pr2_controller_manager" type="spawner" args="wsg_50_gl" name="wsg_50_gl" />
    # Controller spawners
    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',  
    )

    right_controller_file = os.path.join(
        get_package_share_directory('wsg_50_simulation'),
        'controllers',
        'wsg_50_gr.yaml'
    )
    base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "wsg_50_gr",
            '--param-file', right_controller_file,
            ],
    )


    return LaunchDescription([
        node_robot_state_publisher,
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                      'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])],),
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[
                  base_controller_spawner,
                        ],
            )
        ),
        ignition_spawn_entity,
        rviz_node,
    ])

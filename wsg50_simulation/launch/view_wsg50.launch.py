
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
        get_package_share_directory('wsg50_simulation'),
        'urdf',
        'wsg50.urdf.xacro'
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

    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
    )





    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])

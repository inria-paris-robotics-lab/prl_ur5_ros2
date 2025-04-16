############################################################################################################
# Description: This launch file is used to start MoveIt! And configure it for the UR5 workbench.
#              It also need the simulation or real rbot launch file to be running.
# Arguments:
#   - use_sim_time: If true, the simulation time will be used. Default is true.
# Usage:
#   $ ros2 launch prl_ur5_moveit start_moveit.launch.py
############################################################################################################
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ############################ MoveIt Part ################################

    #Config file
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="prl_ur5_moveit")
        .robot_description_semantic(Path("config") / "ur5.srdf")
        .to_moveit_configs()
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("prl_ur5_moveit"), "config", "moveit.rviz"]
    )

    # Launch MoveIt    
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": "true",
            },
        ],
    )
    # Load MoveIt's controllers
    moveit_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('prl_ur5_moveit'),
            'launch',
            'spawn_controllers.launch.py',
            ])
        ]),
    )

    # virtual_joint = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #         FindPackageShare('prl_ur5_moveit'),
    #         'launch',
    #         'static_virtual_joint_tfs.launch.py',
    #         ])
    #     ]),
    # )
    # rsp= IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #         FindPackageShare('prl_ur5_moveit'),
    #         'launch',
    #         'rsp.launch.py',
    #         ])
    #     ]),
    # )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    return LaunchDescription([
        wait_robot_description,
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[
                    node_move_group,
                    # moveit_controller,
                    rviz_node,
                        ],
            )
        ),
    ])

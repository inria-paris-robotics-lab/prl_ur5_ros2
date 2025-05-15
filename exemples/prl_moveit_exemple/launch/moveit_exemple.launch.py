"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mantis", package_name="prl_ur5_moveit")
        .robot_description(file_path="config/mantis.urdf.xacro")
        
        .moveit_cpp(
            file_path=get_package_share_directory("prl_moveit_exemple")
            + "/config/planner.yaml"
        )
        .to_moveit_configs()
    )



    moveit_py_node = Node(
        name="moveit_cmd",
        package="prl_moveit_exemple",
        executable="moveit_cmd",
        output="both",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],

    )

    return LaunchDescription(
        [
            moveit_py_node,
        ]
    )

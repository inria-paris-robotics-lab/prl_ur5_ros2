from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([

    Node(
      package='controller_manager',
      executable='spawner',
      output='both',
      arguments=["joint_state_broadcaster"]

    ),
    Node(
      package='controller_manager',
      executable='spawner',
      output='both',
      arguments=["forward_velocity_controller",
            '--param-file', PathJoinSubstitution([FindPackageShare('prl_ur5_control'),'config', 'dual_ur_controller.yaml']),]
    ),
  ])

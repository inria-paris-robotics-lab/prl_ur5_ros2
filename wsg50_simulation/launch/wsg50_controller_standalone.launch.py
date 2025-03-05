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
    # Node(
    #   package='controller_manager',
    #   executable='spawner',
    #   output='both',
    #   arguments=["position_controller"]
    # ),
    Node(
      package='controller_manager',
      executable='spawner',
      output='both',
      arguments=["wsg_50_gl"]
    ),
    Node(
      package='controller_manager',
      executable='spawner',
      output='both',
      arguments=["wsg_50_gr"]
    ),
  ])

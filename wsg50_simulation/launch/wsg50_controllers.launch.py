from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  wsg50_controller_file=PathJoinSubstitution([FindPackageShare('wsg50_simulation'),'controllers', 'wsg50_integrate.yaml'])
    
  right_wsg50_controller_spawner_gr = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['right_wsg_50_gr', '--param-file', wsg50_controller_file],
  )

  right_wsg50_controller_spawner_gl = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['right_wsg_50_gl', '--param-file', wsg50_controller_file],
  )

  left_wsg50_controller_spawner_gr = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['left_wsg_50_gr', '--param-file', wsg50_controller_file],
  )

  left_wsg50_controller_spawner_gl = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['left_wsg_50_gl', '--param-file', wsg50_controller_file],
  )


  return LaunchDescription([
    right_wsg50_controller_spawner_gr,
    right_wsg50_controller_spawner_gl,
    left_wsg50_controller_spawner_gr,
    left_wsg50_controller_spawner_gl
  ])

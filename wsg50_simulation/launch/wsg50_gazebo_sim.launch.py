from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    description_file = PathJoinSubstitution([FindPackageShare('wsg50_simulation'), 'urdf', 'wsg50.urdf.xacro'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
        ]
    )

    robot_description = {'robot_description':  ParameterValue(value=robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='both',
      parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('ros_gz_sim'),
          'launch',
          'gz_sim.launch.py',
        ])
      ]),
      launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    spawn_gripper = Node(
      package='ros_gz_sim',
      executable='create',
      output='screen',
      arguments=['-topic', 'robot_description',
                 '-name', 'wsg50',
                 '-allow_renaming', 'true'],
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('wsg50_simulation'),
            'launch',
            'wsg50_controller_standalone.launch.py',
            ])
        ]),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated clock'),
        gz_sim_bridge,
        node_robot_state_publisher,
        rviz,
        gazebo_launch,
        spawn_gripper,
        controller_launch,
    ])
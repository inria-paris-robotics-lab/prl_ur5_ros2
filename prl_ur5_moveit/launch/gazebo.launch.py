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
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    ############################ Gazebo Part ################################

    # Config file
    description_file=PathJoinSubstitution([FindPackageShare('prl_ur5_description'),'urdf', 'workbench_ur5.urdf.xacro'])
    controller_file=PathJoinSubstitution([FindPackageShare('prl_ur5_moveit'),'config', 'ros2_controllers.yaml'])
    world_file=PathJoinSubstitution([FindPackageShare('prl_ur5_gazebo'),'world', 'default.sdf'])

    # Load robot
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            description_file,
            " ",
            "gz_sim:=",
            "true",
            " ",
            "controller_file:=",
            controller_file,
            " ",
        ]
    )

    # Wrap robot_description in ParameterValue
    robot_description = {'robot_description':  ParameterValue(value=robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Launch Gazebo*
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                      'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ['-r -v 4 ',world_file])],)

    # Spawn entity in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'dual_ur',
                   '-allow_renaming', 'true'],
        parameters=[{"use_sim_time": use_sim_time}],
    )
  
    # Spawn wsg50's controllers
    wsg50_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('wsg50_simulation'),
            'launch',
            'wsg50_controllers.launch.py',
            ])
        ]),
    )

    # Bridge between ROS and Gazebo (share the same clock it's important to sync the simulation and moveit)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/left_force_torque@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/right_force_torque@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

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
    


    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[
                    node_move_group,
                    moveit_controller,
                    rviz_node,
                        ],
            )
        ),
        ignition_spawn_entity,
    ])

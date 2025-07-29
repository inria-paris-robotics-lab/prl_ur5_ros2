import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import argparse

def generate_camera_bridge_config(config_filepath, output_filepath):
    """
    Generates a configuration file for the ROS-Gazebo bridge based on
    the activation options of each sensor specified in the camera configuration file.

    Args:
        config_filepath (Path or str): Path to the camera configuration YAML file.
        output_filepath (str): Path to the output bridge configuration file.

    This function reads the camera configuration, checks which cameras and features are enabled,
    and writes the corresponding ROS-Gazebo bridge config file.
    """
    base_gz_path = "/world/mantis_world/model/mantis/link/base_link/sensor/"

    try:
        with open(config_filepath, 'r') as f:
            config_data = yaml.safe_load(f)
            cameras = config_data.get('cameras', {})
    except FileNotFoundError:
        print(f"Erreur : Le fichier de configuration '{config_filepath}' n'a pas été trouvé.")
        exit(1)
    except yaml.YAMLError as e:
        print(f"Erreur de syntaxe dans le fichier YAML '{config_filepath}': {e}")
        exit(1)

    all_topics = []
    activated_cameras = []

    for camera_name, camera_info in cameras.items():
        if camera_info.get('activate'):
            activated_cameras.append(camera_name)
            print(f"Processing activated camera: {camera_name}")

            # RGB image bridge
            if camera_info.get('enable_color'):
                print("  - Color camera enabled ")
                all_topics.append({
                    'ros_topic_name': f'camera/{camera_name}/color/image',
                    'gz_topic_name': f'{base_gz_path}{camera_name}_color/image',
                    'ros_type_name': 'sensor_msgs/msg/Image',
                    'gz_type_name': 'gz.msgs.Image',
                    'direction': 'GZ_TO_ROS'
                })
                all_topics.append({
                    'ros_topic_name': f'camera/{camera_name}/color/camera_info',
                    'gz_topic_name': f'{base_gz_path}{camera_name}_color/camera_info',
                    'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                    'gz_type_name': 'gz.msgs.CameraInfo',
                    'direction': 'GZ_TO_ROS'
                })

            # Depth camera bridge
            if camera_info.get('enable_depth'):
                print("  - Depth camera enabled ")
                all_topics.append({
                    'ros_topic_name': f'camera/{camera_name}/depth/camera_info',
                    'gz_topic_name': f'{base_gz_path}{camera_name}_depth/camera_info',
                    'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                    'gz_type_name': 'gz.msgs.CameraInfo',
                    'direction': 'GZ_TO_ROS'
                })
                all_topics.append({
                    'ros_topic_name': f'camera/{camera_name}/depth/depth_image',
                    'gz_topic_name': f'{base_gz_path}{camera_name}_depth/depth_image',
                    'ros_type_name': 'sensor_msgs/msg/Image',
                    'gz_type_name': 'gz.msgs.Image',
                    'direction': 'GZ_TO_ROS'
                })

            # Pointcloud bridge
            if camera_info.get('pointcloud'):
                print("  - Pointcloud enabled ")
                all_topics.append({
                    'ros_topic_name': f'camera/{camera_name}/depth/depth_image/points',
                    'gz_topic_name': f'{base_gz_path}{camera_name}_depth/depth_image/points',
                    'ros_type_name': 'sensor_msgs/msg/PointCloud2',
                    'gz_type_name': 'gz.msgs.PointCloudPacked',
                    'direction': 'GZ_TO_ROS'
                })

            # Infrared sensors bridge
            if camera_info.get('enable_infra'):
                print("  - Infrared cameras enabled ")
                if camera_info.get('enable_infra1'):
                    print("    - Infrared camera 1 enabled ")
                    all_topics.append({
                        'ros_topic_name': f'camera/{camera_name}/ired/ired1/image',
                        'gz_topic_name': f'{base_gz_path}{camera_name}_ired1/image',
                        'ros_type_name': 'sensor_msgs/msg/Image',
                        'gz_type_name': 'gz.msgs.Image',
                        'direction': 'GZ_TO_ROS'
                    })
                    all_topics.append({
                        'ros_topic_name': f'camera/{camera_name}/ired/ired1/camera_info',
                        'gz_topic_name': f'{base_gz_path}{camera_name}_ired1/camera_info',
                        'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                        'gz_type_name': 'gz.msgs.CameraInfo',
                        'direction': 'GZ_TO_ROS'
                    })
                if camera_info.get('enable_infra2'):
                    print("    - Infrared camera 2 enabled ")
                    all_topics.append({
                        'ros_topic_name': f'camera/{camera_name}/ired/ired2/image',
                        'gz_topic_name': f'{base_gz_path}{camera_name}_ired2/image',
                        'ros_type_name': 'sensor_msgs/msg/Image',
                        'gz_type_name': 'gz.msgs.Image',
                        'direction': 'GZ_TO_ROS'
                    })
                    all_topics.append({
                        'ros_topic_name': f'camera/{camera_name}/ired/ired2/camera_info',
                        'gz_topic_name': f'{base_gz_path}{camera_name}_ired2/camera_info',
                        'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                        'gz_type_name': 'gz.msgs.CameraInfo',
                        'direction': 'GZ_TO_ROS'
                    })
        else:
            print(f"Skipping deactivated camera: {camera_name}")


    # Write the configuration to the output file
    try:
        with open(output_filepath, 'w') as f:
            yaml.dump(all_topics, f, sort_keys=False, default_flow_style=False, indent=2)
        
        if activated_cameras:
            print(f"\n Configuration file '{output_filepath}' successfully generated for cameras: {', '.join(activated_cameras)}.")
        else:
            print(f"\n Configuration file '{output_filepath}' generated (it is empty because no camera is activated).")

    except IOError as e:
        print(f"Error: Unable to write to the file '{output_filepath}': {e}")


if __name__ == "__main__":
    # Define the default path
    default_input_path = Path(get_package_share_directory('prl_ur5_robot_configuration')) / 'config/fixed_cameras/cameras_config.yaml'

    # Command-line interface definition
    parser = argparse.ArgumentParser(
        description="Generates a configuration file for the ROS-Gazebo bridge from a camera configuration file.",
    )
    parser.add_argument(
        '-i', '--input',
        type=Path,
        default=default_input_path,
        help=f"Path to the camera configuration file.\n(DDefault: {default_input_path})"
    )
    parser.add_argument(
        '-o', '--output',
        type=str,
        default="generated_ros_gz_bridge.yaml",
        help="Path to the output bridge configuration file.\n(DDefault: 'generated_ros_gz_bridge.yaml')"
    )
    args = parser.parse_args()
    generate_camera_bridge_config(args.input, args.output)

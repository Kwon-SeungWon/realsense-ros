import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def check_param_file(context, *args, **kwargs):
    param_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []


def generate_launch_description():
    # Load shared directories
    amr_core_dir = get_package_share_directory('amr_core')
    amr_description_dir = get_package_share_directory('amr_description')
    amr_serial_dir = get_package_share_directory('amr_serial')
    # amr_lidar_merge_dir = get_package_share_directory('ros2_laser_merger')

    # Paths to parameter and URDF files
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='src/redbot_ros2_project/parameter_file/redbot.yaml',
        description='Relative path to the unified parameter YAML file'
    )
    lidar_declare_params = DeclareLaunchArgument('lidar_params_file',
        default_value=os.path.join(
        amr_core_dir, 'param', 'ydlidar.yaml'),
        description='FPath to the ROS2 parameters file to use.'
    )

    param_file = LaunchConfiguration('params_file')
    lidar_param_file = LaunchConfiguration('lidar_params_file')

    urdf_path = os.path.join(amr_description_dir, 'urdf', 'redbot04.urdf')

    # Read robot_description
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        declare_params,
        lidar_declare_params,
        OpaqueFunction(function=check_param_file),  # (optional) path check
        # Declare launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # AMR core node
        Node(
            package='amr_core',
            executable='amr_core_node',
            name='amr_core',
            output='screen',
            parameters=[param_file]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_param_file],
            namespace='/',
                                )
        # Joint State Publisher
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen'
        # ),

        ## Serial Node
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(amr_serial_dir, 'launch', 'wheelchair_serial.launch.py')
        #    )
        #),

        # # Include Dual LiDAR launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(amr_core_dir, 'launch', 'dual_picoscan.launch.py')
        #     )
        # ),

        # # Include Merge LiDAR launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(amr_lidar_merge_dir, 'launch', 'merge_scan.launch.py')
        #     )
        # ),
    ])
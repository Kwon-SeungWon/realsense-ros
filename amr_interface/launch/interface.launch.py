import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def check_param_file(context, *args, **kwargs):
    param_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []

def generate_launch_description():

    # Declare a relative default path from the workspace root (not from install)
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='src/redbot_ros2_project/parameter_file/redbot.yaml',
        description='Relative path to the unified parameter YAML file'
    )

    param_file = LaunchConfiguration('params_file')

    # Include waypoint_node_manager launch file
    waypoint_node_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('waypoint_node_manager'),
                'launch',
                'waypoint_node_manager.launch.py'
            )
        ])
    )

    return LaunchDescription([
        declare_params,
        OpaqueFunction(function=check_param_file),  # (optional) path check
        # Interface Node
        Node(
            package='amr_interface',
            executable='interface_node',
            output='screen',
            parameters=[param_file],
        ),
        # Include waypoint_node_manager
        waypoint_node_manager_launch,
        # Parameter Updater Node
        # Node(
        #     package='amr_interface',
        #     executable='parameter_updater_node',
        #     name='parameter_updater_node',
        #     output='screen',
        #     parameters=[{'yaml_file': updater_params_file}],
        # ),
    ])

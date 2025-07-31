#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Bishop Pearson

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def check_param_file(context, *args, **kwargs):
    param_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []

def generate_launch_description():
    # Current Package Path
    launch_file_dir = os.path.join(
        get_package_share_directory("amr_navigation"), "launch"
    )

    # Declare launch configuration variables with default values
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("amr_navigation"), "maps", "skku_7th.yaml"
        ),
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='src/redbot_ros2_project/parameter_file/redbot.yaml',
        description='Relative path to the unified parameter YAML file'
    )

    param_file = LaunchConfiguration('params_file')
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    remappings = LaunchConfiguration('remappings', default="[('/cmd_vel', '/nav_vel')]")

    # Add the periodic costmap clearer node
    # costmap_clearer_node = Node(
    #     package='amr_navigation',
    #     executable='periodic_costmap_clearer_node',
    #     name='periodic_costmap_clearer_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    return LaunchDescription(
        [
            declare_params,
            OpaqueFunction(function=check_param_file),  # (optional) path check
            # Declare the arguments with descriptions
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),

            # Include the navigation bringup launch file with configured arguments
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": use_sim_time,
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),

            # Add the periodic costmap clearer node
            # costmap_clearer_node,

            # Uncomment this to include the RViz launch file if needed
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_file_dir, "navigation_rviz.launch.py")
            #     )
            # ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

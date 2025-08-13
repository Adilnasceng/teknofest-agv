#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package path
    diffdrive_arduino_pkg = get_package_share_directory('diffdrive_arduino')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

# Launch dosyasında keepout_mask_file_arg kısmını şu şekilde değiştirin:
    keepout_mask_file_arg = DeclareLaunchArgument(
    'keepout_mask_file',
        default_value='/home/ubuntu/denem/diffdrive_arduino/bringup/maps/keepout_mask.yaml',
        description='Full path to keepout mask yaml file'
)

    # Costmap Filter Info Server
    filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'type': 0,
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/filter_mask',
                'base': 0.0,
                'multiplier': 1.0
            }
        ]
    )

    # Filter Mask Server
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('keepout_mask_file'),
                'topic_name': '/filter_mask',
                'frame_id': 'map'
            }
        ]
    )

    # Lifecycle Manager for Filter Servers
    lifecycle_manager_filters = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_filters',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['filter_info_server', 'filter_mask_server']
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        keepout_mask_file_arg,
        filter_info_server,
        filter_mask_server,
        lifecycle_manager_filters
    ])
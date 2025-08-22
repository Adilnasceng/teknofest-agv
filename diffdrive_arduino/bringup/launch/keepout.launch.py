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

    # Keepout mask file argument - mevcut haritanızı kullan
    keepout_mask_file_arg = DeclareLaunchArgument(
        'keepout_mask_file',
        default_value='/home/ubuntu/denem/diffdrive_arduino/bringup/maps/keepout_mask.yaml',
        description='Full path to keepout mask yaml file'
    )

    # Keepout Filter Info Server
    keepout_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'type': 0,  # Keepout filter type
                'filter_info_topic': '/keepout_filter_info',
                'mask_topic': '/keepout_mask',
                'base': 0.0,
                'multiplier': 1.0
            }
        ]
    )

    # Keepout Mask Server
    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_mask_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('keepout_mask_file'),
                'topic_name': '/keepout_mask',
                'frame_id': 'map'
            }
        ]
    )

    # Lifecycle Manager for Keepout Filter Servers
    lifecycle_manager_keepout = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['keepout_filter_info_server', 'keepout_mask_server']
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        keepout_mask_file_arg,
        keepout_filter_info_server,
        keepout_mask_server,
        lifecycle_manager_keepout
    ])
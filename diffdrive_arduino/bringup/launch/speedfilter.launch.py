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

    # Speed mask file argument - mevcut haritanızı kullan
    speed_mask_file_arg = DeclareLaunchArgument(
        'speed_mask_file',
        default_value='/home/ubuntu/denem/diffdrive_arduino/bringup/maps/speed_mask.yaml',  # Mevcut haritanızı kullan
        description='Full path to speed mask yaml file'
    )

    # Speed Filter Info Server
    speed_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'type': 1,  # Speed filter type
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_mask',
                'base': 0.0,    # Base speed limit (%)
                'multiplier': 0.9  # Speed multiplier
            }
        ]
    )

    # Speed Mask Server
    speed_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_mask_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('speed_mask_file'),
                'topic_name': '/speed_mask',
                'frame_id': 'map'
            }
        ]
    )

    # Lifecycle Manager for Speed Filter Servers
    lifecycle_manager_speed = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_speed',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['speed_filter_info_server', 'speed_mask_server']
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        speed_mask_file_arg,
        speed_filter_info_server,
        speed_mask_server,
        lifecycle_manager_speed
    ])
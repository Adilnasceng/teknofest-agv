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

    # Keepout mask file argument
    keepout_mask_file_arg = DeclareLaunchArgument(
        'keepout_mask_file',
        default_value='/home/ubuntu/denem/diffdrive_arduino/bringup/maps/keepout_mask.yaml',
        description='Full path to keepout mask yaml file'
    )

    # Speed limit mask file argument
    speed_limit_mask_file_arg = DeclareLaunchArgument(
        'speed_limit_mask_file',
        default_value='/home/ubuntu/denem/diffdrive_arduino/bringup/maps/speed_mask.yaml',
        description='Full path to speed limit mask yaml file'
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
                'type': 0,  # Keepout filter için type: 0
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/filter_mask',
                'base': 0.0,
                'multiplier': 1.0
            }
        ]
    )

    # Keepout Filter Mask Server
    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
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

    # Speed Limit Filter Info Server
    speed_limit_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_limit_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'type': 1,  # Speed limit filter için type: 1
                'filter_info_topic': '/speed_limit_filter_info',
                'mask_topic': '/speed_limit_mask',
                'base': 0.26,  # Temel hız limiti (m/s)
                'multiplier': 1.0
            }
        ]
    )

    # Speed Limit Filter Mask Server
    speed_limit_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_limit_filter_mask_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('speed_limit_mask_file'),
                'topic_name': '/speed_limit_mask',
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
                'node_names': [
                    'keepout_filter_info_server', 
                    'keepout_filter_mask_server',
                    'speed_limit_filter_info_server',
                    'speed_limit_filter_mask_server'
                ]
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        keepout_mask_file_arg,
        speed_limit_mask_file_arg,
        keepout_filter_info_server,
        keepout_filter_mask_server,
        speed_limit_filter_info_server,
        speed_limit_filter_mask_server,
        lifecycle_manager_filters
    ])
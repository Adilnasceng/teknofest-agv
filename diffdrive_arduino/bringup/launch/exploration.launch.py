#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Exploration Node - mevcut sisteme eklenir
    exploration_node = Node(
        package='diffdrive_arduino',
        executable='exploration_node.py',
        name='exploration_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            
            # Exploration parametreleri
            'enable_exploration': True,
            'exploration_radius': 3.0,              # Keşif yarıçapı (metre)
            'min_frontier_size': 0.4,               # Min frontier boyutu (metre)
            'goal_timeout': 35.0,                   # Hedef timeout (saniye)
            'random_walk_probability': 0.1,         # Random walk olasılığı
            'safe_distance': 0.9,                   # Güvenli mesafe (metre)
            'exploration_speed': 0.25,              # Keşif hızı (m/s)
            'map_resolution': 0.05,                 # Harita çözünürlüğü
            'frontier_detection_threshold': 0.1,    # Frontier tespit eşiği
            'return_to_start': True,                # Başlangıç pozisyonuna dön
            'start_position_tolerance': 0.5,       # Başlangıç pozisyon toleransı (metre)
        }],
        remappings=[
            # Mevcut sisteminizin topic'leri ile eşleştir
            ('/goal_pose', '/goal_pose'),
            ('/map', '/map'),
            ('/scan', '/scan'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/navigate_to_pose/_action/status', '/navigate_to_pose/_action/status'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        exploration_node
    ])
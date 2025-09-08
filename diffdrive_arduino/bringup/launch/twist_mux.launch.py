#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Config dosyasının yolu
    twist_mux_config = os.path.join(
        get_package_share_directory('diffdrive_arduino'),
        'config',
        'twist_mux.yaml'
    )

    # Twist Mux Node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Twist mux çıkışını obstacle wait'e gönder
            ('/cmd_vel', '/cmd_vel_filtered')
        ]
    )

    # Obstacle Wait Node - Artık bu launch'ta değil, ayrı çalışacak
    # çünkü obstacle wait node kendi topic'lerini kullanıyor
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        
        twist_mux_node,
        # obstacle_wait_node  # KALDIRILDI - ayrı launch edilecek
    ])


# GÜNCELLENMIŞ SİSTEM MİMARİSİ:
#
# Navigation Stack → /cmd_vel_nav ↘
# Task Manager → /cmd_vel_task → Twist Mux → /cmd_vel_filtered → Obstacle Wait → Robot
# Teleop → /cmd_vel_teleop ↗
#
# Obstacle Wait Node artık:
# - Twist mux çıkışından /cmd_vel_filtered'ı dinliyor
# - Tüm kaynaklardan gelen komutları engelleyebiliyor
# - Robot kontrolcüsüne /diffbot_base_controller/cmd_vel_unstamped gönderebiliyor
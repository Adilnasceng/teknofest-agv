#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package path
    diffdrive_arduino_pkg = get_package_share_directory('diffdrive_arduino')
    
    # Twist mux config file path
    twist_mux_params = os.path.join(
        diffdrive_arduino_pkg,
        'config',
        'twist_mux.yaml'
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Twist Mux Node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cmd_vel_out', '/cmd_vel')  # Çıkış olarak /cmd_vel kullan
        ]
    )

    # Line Follower Node
    line_follower_node = Node(
        package='diffdrive_arduino',
        executable='line_follower_node.py',
        name='line_follower_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Dynamic Goal Task Manager Node - ÇİZGİ ALGILAMA BAZLI
    goal_task_manager_node = Node(
        package='diffdrive_arduino',
        executable='dynamic_goal_task_manager_node.py',
        name='dynamic_goal_task_manager',
        output='screen',
        parameters=[{
            'total_goals': 1,
            
            # Çizgi takibi parametreleri (SÜRE BAZLI KALDIRILDI)
            'line_lost_timeout': 3.0,        # Çizgi kaybolma timeout süresi (saniye)
            'line_status_check_rate': 0.1,   # Çizgi durumu kontrol frekansı
            
            # Özel hareket parametreleri
            'forward_speed': 0.2,            # Kutu alma için ileri hız
            'forward_duration': 3.0,         # Kutu alma için ileri süresi
            'turn_speed': 0.5,               # Kutu bırakma için dönüş hızı (rad/s)
            'turn_duration': 3.14,           # Kutu bırakma için 180° dönüş süresi (pi saniye)
            
            # Genel parametreler
            'task_delay': 2.0,
            'post_task_wait': 5.0,
            'return_to_start': True,
            'debug_mode': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        twist_mux_node,          # Twist mux controller
        line_follower_node,      # Çizgi takibi node'u
        goal_task_manager_node   # Ana görev yöneticisi (Çizgi algılama bazlı)
    ])
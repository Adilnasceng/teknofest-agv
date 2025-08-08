#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Obstacle Wait Node - buzzer kontrolü ile
    obstacle_wait_node = Node(
        package='diffdrive_arduino',
        executable='obstacle_wait_node.py',
        name='obstacle_wait_node',
        output='screen',
        parameters=[{
            'obstacle_distance_threshold': 0.8,     # Engel tespit mesafesi (metre)
            'obstacle_angle_range': 60.0,           # Engel tespit açı aralığı (derece, önde ±30°)
            'wait_duration': 15.0,                  # Engel tespit edildiğinde bekleme süresi (saniye)
            'min_scan_points': 5,                   # Minimum engel noktası sayısı
            'enable_obstacle_wait': True,           # Engel bekleme özelliğini aktif et
            'ignore_duration': 30.0,                # Bekleme sonrası ignore süresi (saniye)
            'enable_buzzer': True                   # Buzzer kontrolünü aktif et
        }]
    )

    # Buzzer Control Node - hardware interface ile konuşan node
    buzzer_control_node = Node(
        package='diffdrive_arduino',
        executable='buzzer_control_node.py',
        name='buzzer_control_node',
        output='screen'
    )

    return LaunchDescription([
        obstacle_wait_node,
        buzzer_control_node
    ])
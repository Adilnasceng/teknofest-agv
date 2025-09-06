#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Servo Control Test Node - servo kontrolünü test etmek için
    servo_test_node = Node(
        package='diffdrive_arduino',
        executable='servo_test_node.py',
        name='servo_test_node',
        output='screen',
        parameters=[{
            'test_interval': 30.0,  # 30 saniyede bir test
            'enable_auto_test': True,  # Otomatik test kapalı
            'debug_mode': True,
        }]
    )

    return LaunchDescription([
        servo_test_node
    ])
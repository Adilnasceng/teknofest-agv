#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch argümanları
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/qr_reader',  # Robot /dev/ttyACM0 kullanıyor, QR için farklı port
        description='QR code okuyucu serial port yolu (örn: /dev/ttyUSB0, /dev/ttyUSB1)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Serial port baud rate (9600, 115200, vb.)'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Serial okuma timeout süresi (saniye)'
    )
    
    # QR Code Reader node
    qr_reader_node = Node(
        package='diffdrive_arduino',  # Robotun paket adı
        executable='qr_code_reader_node.py',  # Python script executable
        name='qr_code_reader',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': LaunchConfiguration('timeout')
        }],
        remappings=[
            ('qr_code_data', '/robot/qr_code_data')  # Robot topic namespace
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        timeout_arg,
        qr_reader_node
    ])
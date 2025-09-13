#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rfid',
        description='Arduino serial port'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Serial baud rate'
    )
    
    # RFID Reader Node
    rfid_reader_node = Node(
        package='diffdrive_arduino',
        executable='rfid_reader_node.py',
        name='rfid_reader_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        remappings=[
            ('/rfid_data', '/sensors/rfid_data'),
            ('/rfid_status', '/sensors/rfid_status')
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        rfid_reader_node
    ])
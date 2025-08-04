from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paket yolları
    rplidar_ros_pkg = os.path.join(get_package_share_directory('rplidar_ros'))
    diffdrive_arduino_pkg = os.path.join(get_package_share_directory('diffdrive_arduino'))
    nav2_bringup_pkg = os.path.join(get_package_share_directory('nav2_bringup'))

    # Serial port argümanı (lazım olur)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )

    # 1. diffbot.launch.py çalışacak
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diffdrive_arduino_pkg, 'launch', 'diffbot.launch.py')
        )
    )

    # 2. 3 saniye sonra lidar başlasın
    rplidar_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_a2m12_launch.py')
                ),
                launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
            )
        ]
    )

    # 3. 5 saniye sonra navigation.launch.py başlasın (params_file ARG'si vermeden)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        diffbot_launch,
        rplidar_launch,
        nav2_launch
    ])

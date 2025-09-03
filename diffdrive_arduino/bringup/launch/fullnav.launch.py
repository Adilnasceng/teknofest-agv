from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paket yolları
    diffdrive_arduino_pkg = os.path.join(get_package_share_directory('diffdrive_arduino'))

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
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'rplidar.launch.py')
                )
            )
        ]
    )

    # 3. 5 saniye sonra localization.launch.py başlasın
    localization = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'localization.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        diffbot_launch,
        rplidar_launch,
        localization
    ])
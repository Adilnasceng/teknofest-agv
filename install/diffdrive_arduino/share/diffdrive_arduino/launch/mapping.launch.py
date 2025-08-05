from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    rplidar_ros_pkg = os.path.join(get_package_share_directory('rplidar_ros'))
    diffdrive_arduino_pkg = os.path.join(get_package_share_directory('diffdrive_arduino'))

    # Declare serial port argument for RPLidar
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    # Launch Diffbot FIRST
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diffdrive_arduino_pkg, 'launch', 'diffbot.launch.py')
        )
    )

    # Delay 3 seconds after diffbot starts, then launch the rest
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

    slam_launch = TimerAction(
        period=3.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'slam.launch.py')
                )
            )
        ]
    )

    joystick_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'joystick.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        diffbot_launch,  # Önce bu başlar
        rplidar_launch,  # 3 saniye sonra lidar başlar
        slam_launch,     # 3.5 saniye sonra slam başlar
        joystick_launch  # 4 saniye sonra joystick başlar
    ])

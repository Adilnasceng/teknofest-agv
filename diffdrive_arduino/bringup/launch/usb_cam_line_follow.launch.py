from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 320,
                'image_height': 240,  # veya 320 kare format için
                'framerate': 15.0,
                'pixel_format': 'yuyv',
                'brightness': 50,
                'contrast': 50,
                'sharpness': 80,
                'autoexposure': False,
                'exposure': 150
            }],
            remappings=[
                ('/image_raw', '/line_camera/image_raw'),
            ]
        )
    ])
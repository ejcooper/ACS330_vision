from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usbcam',
            parameters=[
            	#{'camera_info_url': '/home/uos/Documents/ACS330_Group_1_files/camera_info/default_cam.yaml'},
                {'image_height': 1080},
                {'image_width': 1920},
                {'framerate': 10.0},
                {'brightness': 100},
                {'autofocus': False}
            ]
        ),
  ])

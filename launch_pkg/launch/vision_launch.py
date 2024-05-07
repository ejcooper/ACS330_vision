from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    aruco_params = {
        'image_is_rectified': True,
        'normalizeImage': False,
        'dct_components_to_remove': 2,
        'parent_name': 'stereo_gazebo_left_camera_optical_frame',
        'child_name1': 'marker_hand_frame',
        'child_name2': 'marker_object_frame',
    }
    
    aruco_remaps = [
        ('/camera_info', '/camera_info'),
        ('/image', '/image_raw'),
    ]
    
    #define the marker sizes in meters
    small_marker = 0.05
    large_marker = 0.085


    ld =[
        # setup the USB camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usbcam',
            parameters=[
                {'image_height': 1080},
                {'image_width': 1920},
                {'framerate': 10.0},
                {'brightness': 100},
                {'autofocus': False},
                {'camera_info_url': "file:///home/uos/Documents/ACS330_Group_1_files/camera_info/default_cam_22_4.yaml"}
            ]
        ),
        
        # Setup the Arcuo marker detection nodes 
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[ aruco_params, {
                'marker_size': large_marker,
                'marker_id1': 0,
                'marker_id2': 1,
                },  
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker0'),
                ('/aruco_double/pose2', '/aruco/marker1')
            ],
        ),
        
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[aruco_params, {
                'marker_size': small_marker,
                'marker_id1': 3,
                'marker_id2': 4,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker3'),
                ('/aruco_double/pose2', '/aruco/marker4')
            ],
        ),
        
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[aruco_params, {
                'marker_size': small_marker,
                'marker_id1': 7,
                'marker_id2': 9,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker7'),
                ('/aruco_double/pose2', '/aruco/marker9')
            ],
        ),
        
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[aruco_params, {
                'marker_size': small_marker,
                'marker_id1': 10,
                'marker_id2': 11,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker10'),
                ('/aruco_double/pose2', '/aruco/marker11')
            ],
        ),
        
        Node(
            package = 'pose_transform',
            executable = 'poseTransformerNode',
        )
        
    ]
        
    return LaunchDescription(ld)
  
  
  

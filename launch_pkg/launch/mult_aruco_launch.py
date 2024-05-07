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
    small_marker = 0.06
    large_marker = 0.085

    ld =[
    
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
                'marker_id1': 2,
                'marker_id2': 3,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker2'),
                ('/aruco_double/pose2', '/aruco/marker3')
            ],
        ),
        
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[aruco_params, {
                'marker_size': small_marker,
                'marker_id1': 4,
                'marker_id2': 5,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker4'),
                ('/aruco_double/pose2', '/aruco/marker5')
            ],
        ),
        
        Node(
            package='aruco_ros',
            executable='double',
            parameters=[aruco_params, {
                'marker_size': small_marker,
                'marker_id1': 6,
                'marker_id2': 7,
                }, 
            ],
            remappings=[
                aruco_remaps[0], aruco_remaps[1],
                ('/aruco_double/pose', '/aruco/marker6'),
                ('/aruco_double/pose2', '/aruco/marker7')
            ],
        ),
    ]
        
    return LaunchDescription(ld)
  
  
  

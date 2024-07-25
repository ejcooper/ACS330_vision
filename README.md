# Overview 

This system will use a usb webcam to detect and locate Aruco markers.
The pose of each marker will be returned on a topic in the ‘/transformed’ namespace, followed by the marker that topic is tracking.
E.g. The pose of the Aruco marker with code 1 is published on the ‘/transformed/marker1’ topic. 
The pose of each maker is published as a Pose message type (geometry_msgs/msg/Pose) which contains the position and orientation of a
given marker relative to marker 0, shown below. The position is in cartesian (X,Y,Z) coordinates and the orientation is given in
quaternions in the order x,y,z,w. Each element can be retrieved using the following syntax: Pose.position.X or Pose.orientation.x
All the distances are measured in metres from the centre of each marker.

# Setup

- navigate to the computers root directory
- create a ROS2 workspace and navigate into the /src folder
- clone this repo along with the 'aruco_ros' and 'usb_cam' repos
- navigate back to the workspace and build

full instructions for this can be found on the ROS documentaion website

# Running

To run, use the terminal to go to, then source, the correct workspace. 

    Source install/local_setup.bash

Then run the ‘vision_system_launch’ script.

    Ros2 launch launch_pkg vision_launch.py


This will launch the ‘USB cam’ node, 4 ‘Aruco double’ nodes, and the ‘pose transformer’ node. This will allow the system to track 6 
blocks as well as the 2 larger markers ( codes 0, 1) used to find the position of both tables.
The pose for each marker is published every time the system receives a frame from the camera. Current camera frame rate is 10 Hz, so 
this also around the rate at which marker poses are published. 


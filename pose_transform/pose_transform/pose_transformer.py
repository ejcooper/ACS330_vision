#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose


def Pose2Transform( pose: Pose ):
    # converts the ros2 Pose message type into a Homogenous transformation matrix
    
    X = pose.position.x
    Y = pose.position.y
    Z = pose.position.z
    
    # for quaternion in from q = q0 + q1i + q2j + q3k
    # alt: q = w + xi +yj + zk
    
    q0 = pose.orientation.w
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
    
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    trans_matrix = np.array([[r00, r01, r02, X],
                             [r10, r11, r12, Y],
                             [r20, r21, r22, Z],
                             [0.0, 0.0, 0.0, 1.0]])           
    return trans_matrix    


def Transform2Pose( mat ):
    # converts a transfroation matrix to ros2 pose message
    X = mat[0][3]
    Y = mat[1][3]
    Z = mat[2][3]
    
    # convert rotation matrix to quaternion rotation
    trace = mat[0][0] + mat[1][1] + mat[2][2]
    
    if trace > 0:
        S = np.sqrt(trace+1.0) * 2 # S=4*qw 
        qw = 0.25 * S
        qx = (mat[2][1] - mat[1][2]) / S
        qy = (mat[0][2] - mat[2][0]) / S
        qz = (mat[1][0] - mat[0][1]) / S
        
    elif ( (mat[0][0] > mat[1][1]) & (mat[0][0] > mat[2][2]) ):
        S = np.sqrt(1.0 + mat[0][0] - mat[1][1] - mat[2][2]) * 2 # S=4*qx 
        qw = (mat[2][1] - mat[1][2]) / S
        qx = 0.25 * S
        qy = (mat[0][1] + mat[1][0]) / S
        qz = (mat[0][2] + mat[2][0]) / S
        
    elif (mat[1][1] > mat[2][2]):
        S = np.sqrt(1.0 + mat[1][1] - mat[0][0] - mat[2][2]) * 2 # S=4*qy
        qw = (mat[0][2] - mat[2][0]) / S
        qx = (mat[0][1] + mat[1][0]) / S
        qy = 0.25 * S
        qz = (mat[1][2] + mat[2][1]) / S
        
    else : 
        S = np.sqrt(1.0 + mat[2][2] - mat[0][0] - mat[1][1]) * 2 # S=4*qz
        qw = (mat[1][0] - mat[0][1]) / S
        qx = (mat[0][2] + mat[2][0]) / S
        qy = (mat[1][2] + mat[2][1]) / S
        qz = 0.25 * S
    
    pose = Pose()
    
    pose.position.x = X
    pose.position.y = Y
    pose.position.z = Z
    
    pose.orientation.w = qw
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    
    return(pose)


def rotate_90( in_mat ):
    
    rot_mat = np.array([ [0.0, -1.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0],[0.0, 0.0, 1.0, 0.0],[0.0, 0.0, 0.0, 1.0] ])
    
    x = in_mat[0][3]
    y = in_mat[1][3]
    z = in_mat[2][3]
    
    in_mat[0][3] = 0.0
    in_mat[1][3] = 0.0
    in_mat[2][3] = 0.0
    
    out_mat = np.matmul(rot_mat, in_mat)
    
    out_mat[0][3] = x
    out_mat[1][3] = y
    out_mat[2][3] = z
    
    return( out_mat )



class poseTransformNode(Node):
    def __init__(self):
        super().__init__("pose_transformer")
        
        # initalise the camera position as being the origin 
        self.origin_frame = np.identity(4) 
            
        self.origin_subscriber = self.create_subscription(
            Pose, "/aruco/marker0", self.origin_callback, 10)
            
            
        self.pose_1_subscriber = self.create_subscription(
            Pose, "/aruco/marker1", self.pose_1_callback, 10)
        
        self.pose_1_publisher = self.create_publisher(
            Pose, "transformed/marker1", 10)
            
            
        self.pose_A_subscriber = self.create_subscription(
            Pose, "/aruco/marker10", self.pose_A_callback, 10)
        
        self.pose_A_publisher = self.create_publisher(
            Pose, "transformed/blockA", 10)
            
            
        self.pose_B_subscriber = self.create_subscription(
            Pose, "/aruco/marker9", self.pose_B_callback, 10)
        
        self.pose_B_publisher = self.create_publisher(
            Pose, "transformed/blockB", 10)
        
        
        self.pose_C_subscriber = self.create_subscription(
            Pose, "/aruco/marker7", self.pose_C_callback, 10)
        
        self.pose_C_publisher = self.create_publisher(
            Pose, "transformed/blockC", 10)
            
            
        self.pose_D_subscriber = self.create_subscription(
            Pose, "/aruco/marker4", self.pose_D_callback, 10)
        
        self.pose_D_publisher = self.create_publisher(
            Pose, "transformed/blockD", 10)
            
            
        self.pose_E_subscriber = self.create_subscription(
            Pose, "/aruco/marker11", self.pose_E_callback, 10)
        
        self.pose_E_publisher = self.create_publisher(
            Pose, "transformed/blockE", 10)
            
            
        self.pose_F_subscriber = self.create_subscription(
            Pose, "/aruco/marker3", self.pose_F_callback, 10)
        
        self.pose_F_publisher = self.create_publisher(
            Pose, "transformed/blockF", 10)
            
            
        self.get_logger().info("Setup pose tranform node. ")



    def PoseTransformation( self, pose:Pose ):
        orig_from_cam = self.origin_frame
        cam_from_orig = np.linalg.inv( orig_from_cam )
        
        mark_from_cam = Pose2Transform( pose )
        
        mark_from_orig_1 = np.matmul( cam_from_orig, mark_from_cam )
        
        # rotate each block 90 to acount for incorrect CAD files
        # if somethings going wrong, it might be this  
        #mark_from_orig = rotate_90(mark_from_orig_1)
        
       
        return( Transform2Pose( mark_from_orig ) )


    def origin_callback( self, msg: Pose ):
    	# update the origin to be at the location of chosen marker
    	self.origin_frame = Pose2Transform( msg )
    	

    def pose_1_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_1_publisher.publish(return_msg)
    
    def pose_A_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_A_publisher.publish(return_msg)
        
    def pose_B_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_B_publisher.publish(return_msg)
        
    def pose_C_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_C_publisher.publish(return_msg)
    
    def pose_D_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_D_publisher.publish(return_msg)
        
    def pose_E_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_E_publisher.publish(return_msg)
        
    def pose_F_callback( self, msg: Pose ):
        return_msg = self.PoseTransformation( msg )
        self.pose_F_publisher.publish(return_msg)

    
def main( args=None ):
    rclpy.init( args=args )
    tfNode = poseTransformNode()

    rclpy.spin(tfNode)
    
    rclpy.shutdown()
    

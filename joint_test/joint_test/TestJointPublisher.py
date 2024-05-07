#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64


class JointPublisher(Node):
    
    def __init__(self):
        super().__init__('joint_publisher')
        
        time = 0.05 #seconds (20 Hz)
        self.timer = self.create_timer( time, self.timer_callback)
        
        self.via_increment = 0
        
        # [shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3] 
        self.ur10_pointer = 0
        self.ur10_waypoints = [[3.1, -2.0, 1.7,  -1.0, -1.5, 0.0],
                               [2.8, -1.0, 1.2,  -1.2, -1.8, 0.0],
                               [3.1, -2.0, 1.7,  -1.0, -1.5, 0.0],
                               [3.4, -1.2, 2.2,  -2.6, -1.7, 0.0],
                               [3.1, -2.0, 1.7,  -1.0, -1.5, 0.0],
                               [3.6, -1.0, 1.5,  -2.2, -1.9, 0.0]]
                               
                               
        
        # [shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3] 
        self.ur5_pointer = 0
        self.ur5_waypoints = [[-1.70, -1.0, -1.8,  -1.8, 1.4, 0.0],
                              [-2.15, -2.5, -0.9,  -1.1, 1.4, 0.0],
                              [-1.70, -1.0, -1.8,  -1.8, 1.4, 0.0],
                              [-1.1, -2.7, -0.5,  -1.5, 1.7, 0.0]]
                              
        
        
        # UR10 publishers
        self.ur10_shoulderPan_ = self.create_publisher(
            Float64, "/model/ur10/joint/shoulder_pan_joint", 10 )
        
        self.ur10_shoulderLift_ = self.create_publisher(
            Float64, "/model/ur10/joint/shoulder_lift_joint", 10 )
            
        self.ur10_elbow_ = self.create_publisher(
            Float64, "/model/ur10/joint/elbow_joint", 10 )
            
        self.ur10_wrist1_ = self.create_publisher(
            Float64, "/model/ur10/joint/wrist_1_joint", 10 )
        
        self.ur10_wrist2_ = self.create_publisher(
            Float64, "/model/ur10/joint/wrist_2_joint", 10 )
        
        self.ur10_wrist3_ = self.create_publisher(
            Float64, "/model/ur10/joint/wrist_3_joint", 10 )
            
            
        # UR5e publishers
        self.ur5_shoulderLift_ = self.create_publisher(
            Float64, "/model/ur5/joint/shoulder_lift_joint", 10 )
            
        self.ur5_shoulderPan_ = self.create_publisher(
            Float64, "/model/ur5/joint/shoulder_pan_joint", 10 )
            
        self.ur5_elbow_ = self.create_publisher(
            Float64, "/model/ur5/joint/elbow_joint", 10 )
            
        self.ur5_wrist1_ = self.create_publisher(
            Float64, "/model/ur5/joint/wrist_1_joint", 10 )
        
        self.ur5_wrist2_ = self.create_publisher(
            Float64, "/model/ur5/joint/wrist_2_joint", 10 )
        
        self.ur5_wrist3_ = self.create_publisher(
            Float64, "/model/ur5/joint/wrist_3_joint", 10 )
            
            
        
        self.get_logger().info( 'Test joint Node is active' )
        
        
        
    def timer_callback(self): 
    	
        num_incs = 50
    	
        num_ur5_points = len(self.ur5_waypoints)
        num_ur10_points = len(self.ur10_waypoints)
        
        ur5_start = self.ur5_waypoints[ self.ur5_pointer%num_ur5_points ]
        ur5_end = self.ur5_waypoints[ (self.ur5_pointer+1)%num_ur5_points ]
        
        ur10_start = self.ur10_waypoints[ self.ur10_pointer%num_ur10_points ]
        ur10_end = self.ur10_waypoints[ (self.ur10_pointer+1)%num_ur10_points ]
    	
        ur5_angles = [0.0,0.0,0.0, 0.0,0.0,0.0]
        ur10_angles = [0.0,0.0,0.0, 0.0,0.0,0.0]
    	
    	
        for i in range(len(ur5_angles)):
            ur5_angles[i] = ur5_start[i] + (ur5_end[i] - ur5_start[i]) * self.via_increment / num_incs 
    	
        for i in range(len(ur10_angles)):
            ur10_angles[i] = ur10_start[i] + (ur10_end[i] - ur10_start[i]) * self.via_increment / num_incs 
    	
    	
        if self.via_increment <  num_incs:
            self.via_increment += 1
        else:
            self.via_increment = 0
            
            self.ur5_pointer += 1
            if self.ur5_pointer == num_ur5_points:
                   self.ur5_pointer = 0
            
            self.ur10_pointer += 1
            if self.ur10_pointer == num_ur10_points:
                   self.ur10_pointer = 0
        
        print(ur5_angles)
        
        self.publish_UR5( ur5_angles)
        self.publish_UR10( ur10_angles)
        
    
        
        
    def move_UR5(self):
    
        num_points = len(self.ur5_waypoints)
        
        start = self.ur5_waypoints[ self.ur5_pointer%num_points ]
        end = self.ur5_waypoints[ (self.ur5_pointer+1)%num_points ]
        
       
        for i in range(len(start)):
            inc = abs( (end[i] - start[i]) / 50 )
        
            if self.ur5_angles[i] < end[i] - inc:
                self.ur5_angles[i] += inc
            
            elif self.ur5_angles[i] > end[i] + inc:
                self.ur5_angles[i] -= inc
            
            else: 
                self.ur5_pointer += 1
            
                if self.ur5_pointer == num_points:
                   self.ur5_pointer = 0
            
     
        #self.publish_UR5( self.ur5_angles)
        
        
        
    def publish_UR10(self, angles):
        # publish the ur10 joint angles to all its topics 
        shoulderPan = Float64()
        shoulderLift = Float64()
        elbow = Float64()
        
        wrist1 = Float64()
        wrist2 = Float64()
        wrist3 = Float64()
        
        
        shoulderPan.data = tau_round( angles[0] )
        shoulderLift.data = tau_round( angles[1] )
        elbow.data = tau_round( angles[2] )
        
        wrist1.data = tau_round( angles[3] )
        wrist2.data = tau_round( angles[4] )
        wrist3.data = tau_round( angles[5] )
        
        
        self.ur10_shoulderPan_.publish( shoulderPan )
        self.ur10_shoulderLift_.publish( shoulderLift )
        self.ur10_elbow_.publish( elbow )
        
        self.ur10_wrist1_.publish( wrist1 )
        self.ur10_wrist2_.publish( wrist2 )
        self.ur10_wrist3_.publish( wrist3 )
        
        
        
    def publish_UR5(self, angles):
    	# publish the ur5 joint angles to all its topics 
        shoulderPan = Float64()
        shoulderLift = Float64()
        elbow = Float64()
        
        wrist1 = Float64()
        wrist2 = Float64()
        wrist3 = Float64()
        
        
        shoulderPan.data = tau_round( angles[0] )
        shoulderLift.data = tau_round( angles[1] )
        elbow.data = tau_round( angles[2] )
        
        wrist1.data = tau_round( angles[3] )
        wrist2.data = tau_round( angles[4] )
        wrist3.data = tau_round( angles[5] )
        
        
        self.ur5_shoulderPan_.publish( shoulderPan )
        self.ur5_shoulderLift_.publish( shoulderLift )
        self.ur5_elbow_.publish( elbow )
        
        self.ur5_wrist1_.publish( wrist1 )
        self.ur5_wrist2_.publish( wrist2 )
        self.ur5_wrist3_.publish( wrist3 )
        
    
def tau_round( theta ):
    # cap the joint angles so they cant exceed their maximum
    tau = 2*np.pi
    
    if theta > tau:
        theta = tau
        
    elif theta < -tau:
        theta = -tau
    
    return theta


        
def main(args=None):
    rclpy.init(args=args)
    
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    
    joint_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
    

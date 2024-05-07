#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class PosePublisher(Node):
    
    def __init__(self):
        super().__init__('pose_publisher')
        
        self.publisher_ = self.create_publisher(
            Pose, "transformed/marker1", 10)
            
        time = 0.1 #seconds (10 Hz)
        self.timer = self.create_timer( time, self.timer_callback)
        
        self.get_logger().info( 'Test Publisher Node is active' )
        
        
    def timer_callback(self): 
        
        pose = Pose()
        
        pose.position.x = 1.0
        pose.position.y = 1.0
        pose.position.z = 1.0
        
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        self.publisher_.publish( pose )
        
        
        
def main(args=None):
    rclpy.init(args=args)
    
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    
    pose_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
    

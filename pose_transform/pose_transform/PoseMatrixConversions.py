import numpy as np
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


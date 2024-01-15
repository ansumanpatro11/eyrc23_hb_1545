#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
# Team ID:		[ 1545 ]
# Author List:	[ Ansuman, Amit, Abhisek, Surya ]
# Filename:		hb_task_2a_1545_feedback.py
# Functions:    [ image_callback, ArucoDetect ]
# Nodes:		ar_uco_detector


################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
from geometry_msgs.msg import Pose2D
import time

# Initialize the ArUcoDetector class, which is a ROS node.
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        
        # Subscribe to the topic /camera/image_raw to receive image data.
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        
        # Create a publisher for the detected ArUco marker's pose.
        self.pub1 = self.create_publisher(Pose2D, "/detected_aruco_1", 10)
        self.pub2 = self.create_publisher(Pose2D, "/detected_aruco_2", 10)
        self.pub3 = self.create_publisher(Pose2D, "/detected_aruco_3", 10)
      
        # Create a CvBridge instance to convert between ROS image messages and OpenCV images.
        self.bridge = CvBridge()
        
        # Create a Pose2D message to store the detected ArUco marker's pose.
        self.msg = Pose2D()
        
        # Initialize variables to store previous position values.
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        
        self.detected_ids = []  # Store detected marker IDs
        self.detected_corners = []  # Store detected marker corners
        
    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image.
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Resize the image for processing (optional).
        self.cv_image = cv2.resize(self.cv_image, (500, 500), interpolation=cv2.INTER_LINEAR)  
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        
        # Detect ArUco markers in the provided image.
        corners, ids, _ = aruco.detectMarkers(self.cv_image, self.aruco_dict, parameters=self.parameters)
        self.detected_ids = ids
        self.detected_corners = corners
def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()
    
    
    while rclpy.ok:
        
        detected_ids = aruco_detector.detected_ids
        detected_corners = aruco_detector.detected_corners
        
        
        
        # Check if marker ID 1 is detected.
        if 1 in detected_ids:
            idx = list(detected_ids).index(1)
            arr = detected_corners[idx][0]
            
            # Calculate the center coordinates and orientation (theta) of the detected marker.
            centre_x_opencv = sum(arr[:, 0]) / 4
            centre_y_opencv = sum(arr[:, 1]) / 4

            #convert the coordinates into cartesian plane
            centre_x = centre_x_opencv
            centre_y = 500-centre_y_opencv


            # Calculate the orientation angle (theta) of the marker.
            right_centre_x = (float((arr[1, 0] + arr[2, 0]) / 2))
            right_centre_y = 500-float((arr[1, 1] + arr[2, 1]) / 2)            
            theta = math.atan2((-centre_y +right_centre_y) , (-centre_x + right_centre_x))

            if theta<0:
                theta+= 2*math.pi
                
            msg=Pose2D()
            msg.x=centre_x
            msg.y=centre_y
            msg.theta=theta
            aruco_detector.pub1.publish(msg) 
            
            
        if 2 in detected_ids:
            idx = list(detected_ids).index(2)
            arr = detected_corners[idx][0]
            
            # Calculate the center coordinates and orientation (theta) of the detected marker.
            centre_x_opencv = sum(arr[:, 0]) / 4
            centre_y_opencv = sum(arr[:, 1]) / 4

            #convert the coordinates into cartesian plane
            centre_x = centre_x_opencv
            centre_y = 500-centre_y_opencv


            # Calculate the orientation angle (theta) of the marker.
            right_centre_x = (float((arr[1, 0] + arr[2, 0]) / 2))
            right_centre_y = 500-float((arr[1, 1] + arr[2, 1]) / 2)            
            theta = math.atan2((-centre_y +right_centre_y) , (-centre_x + right_centre_x))

            if theta<0:
                theta+= 2*math.pi
                
            msg=Pose2D()
            msg.x=centre_x
            msg.y=centre_y
            msg.theta=theta
            aruco_detector.pub2.publish(msg) 
            
        if 3 in detected_ids:
            idx = list(detected_ids).index(3)
            arr = detected_corners[idx][0]
            
            # Calculate the center coordinates and orientation (theta) of the detected marker.
            centre_x_opencv = sum(arr[:, 0]) / 4
            centre_y_opencv = sum(arr[:, 1]) / 4

            #convert the coordinates into cartesian plane
            centre_x = centre_x_opencv
            centre_y = 500-centre_y_opencv


            # Calculate the orientation angle (theta) of the marker.
            right_centre_x = (float((arr[1, 0] + arr[2, 0]) / 2))
            right_centre_y = 500-float((arr[1, 1] + arr[2, 1]) / 2)            
            theta = math.atan2((-centre_y +right_centre_y) , (-centre_x + right_centre_x))

            if theta<0:
                theta+= 2*math.pi
                
            msg=Pose2D()
            msg.x=centre_x
            msg.y=centre_y
            msg.theta=theta
            aruco_detector.pub3.publish(msg)        
            
            
            
            # print(self.centre_x, self.centre_y, self.theta)
            
            # # Handle cases where center coordinates are None.
            # if centre_x is None:
            #     centre_x = 0.0
            # if centre_y is None:
            #     centre_y = prev_y
        

        rclpy.spin_once(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
          
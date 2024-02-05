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
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
from geometry_msgs.msg import Pose2D
import time
from utils import *

# Initialize the ArUcoDetector class, which is a ROS node.
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('feedback_node')
        
        # Subscribe to the topic /camera/image_raw to receive image data.
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        
        # Create a publisher for the detected ArUco marker's pose.
        self.pub1 = self.create_publisher(Pose2D, "/pen1_pose", 10)
        self.pub2 = self.create_publisher(Pose2D, "/pen2_pose", 10)
        self.pub3 = self.create_publisher(Pose2D, "/pen3_pose", 10)
        
        # Create a CvBridge instance to convert between ROS image messages and OpenCV images.
        self.bridge = CvBridge()
        
        # Create a Pose2D message to store the detected ArUco marker's pose.
        self.msg_1 = Pose2D()
        self.msg_2 =Pose2D()
        self.msg_3 =Pose2D()
        self.msg=[self.msg_1,self.msg_2,self.msg_3]
        self.topleft=None
        self.topright=None
        self.bottomleft=None
        self.bottomright=None
        bot_ids=[1,2,3]
        self.bot_paths = [[] for _ in range(len(bot_ids))]
       
        
            
    

    def image_callback(self,msg):
        # Convert the ROS image message to an OpenCV image.
        self.distorted_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.ArucoDetect(self.image_converter(self.distorted_image)) 
        # Publish the detected ArUco marker's pose.
        if self.msg_1 is not None:
            self.pub1.publish(self.msg_1)
              # Draw dots at the path points
        if self.msg_2 is not None:       
            self.pub2.publish(self.msg_2)
        if self.msg_3 is not None:
            self.pub3.publish(self.msg_3)

    
    def image_converter(self,image):
        # self.caliberated_image=camera_caliberation(image)

        corners,ids=aruco_detect(self.distorted_image)
        self.topleft,self.topright,self.bottomleft,self.bottomright=get_corners(corners,ids)
        if self.topleft is not None and self.topright is not None and self.bottomleft is not None and self.bottomright is not None:
            self.arena_corners=np.float32([self.topleft,self.topright,self.bottomleft,self.bottomright])
            self.transformed_image=perspective_transform(self.distorted_image,self.arena_corners)
            
            return self.transformed_image



        


    def ArucoDetect(self,cv_image):
        # Initialize ArUco marker detection parameters.
        # Detect ArUco markers in the provided image.
        # print("hello")
        # print(cv_imag/e)
        if self.distorted_image is not None:
            cv2.imshow('frame',self.distorted_image)
            cv2.waitKey(1)
        #later modify this to perspective transformed image
            thresholded_image=thresholding(self.distorted_image)
            corners, ids = aruco_detect(thresholded_image)
            print(ids)
            print(len(corners))
            bot_ids=[1,2,3]
            
            # Check if marker ID 1 is detected.
            for i in range(len(corners)):
                for j in range(len(bot_ids)):
                    if ids[i][0]==bot_ids[j]:
                        arr=corners[i][0]
                         # Calculate the center coordinates and orientation (theta) of the detected marker
                        self.centre_x_opencv = sum(arr[:, 0]) / 4
                        self.centre_y_opencv = sum(arr[:, 1]) / 4
                        cv2.circle(cv_image, (int(self.centre_x_opencv), int(self.centre_y_opencv)), 5, (0, 255, 0), -1)
                        self.bot_paths[j].append((self.centre_x_opencv, self.centre_y_opencv))

                # Draw dots at the path points
                        for k in range(len(self.bot_paths[j])):
                            point = self.bot_paths[j][k]
                            cv2.circle(self.distorted_image, (int(point[0]), int(point[1])), 1, (0, 0, 255), -1)
                      
                        #convert the coordinates into cartesian plane
                        self.centre_x = self.centre_x_opencv
                        self.centre_y = 500-self.centre_y_opencv

                         # Calculate the orientation angle (theta) of the marker.
                        right_centre_x = (float((arr[1, 0] + arr[2, 0]) / 2))
                        right_centre_y = 500-float((arr[1, 1] + arr[2, 1]) / 2)            
                        self.theta = math.atan2((self.centre_y - right_centre_y) ,(self.centre_x - right_centre_x))
                        # if self.theta<0:
                        self.theta+=math.pi
                        print(bot_ids[j],self.centre_x, self.centre_y, self.theta)
                        self.msg[j].x=self.centre_x
                        self.msg[j].y=self.centre_y
                        self.msg[j].theta=self.theta
                        print("hi")
                    # for k in range(j+1):
                    #         # if self.msg[k].x is not None and self.msg[k].y is not None:
                    #             cv2.circle(self.distorted_image, (int(self.msg[k].x), int(500-self.msg[k].y)), 5, (0, 0, 255), -1)


            cv2.imshow('frame',self.distorted_image)
            cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    
    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

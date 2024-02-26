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
# Author List:	[ Ansuman, Amit, Abhisek ]
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
from std_msgs.msg import Int32


# Initialize the ArUcoDetector class, which is a ROS node.
class FeedBack(Node):

    def __init__(self):
        super().__init__('feedback_node')
        
        # Subscribe to the topic /camera/image_raw to receive image data.
        self.sub = self.create_subscription(Image, "/camera1/image_raw", self.image_callback, 10)
        
        # Create a publisher for the detected ArUco marker's pose.
        self.pub1 = self.create_publisher(Pose2D, "/pen1_pose", 1)
        self.pub2 = self.create_publisher(Pose2D, "/pen2_pose", 1)
        self.pub3 = self.create_publisher(Pose2D, "/pen3_pose", 1)
        
        self.sub = self.create_subscription(Int32, "/i_1", self.iter_callback_1, 10)
        self.sub = self.create_subscription(Int32, "/i_2", self.iter_callback_2, 10)
        self.sub = self.create_subscription(Int32, "/i_3", self.iter_callback_3, 10)


        
        # Create a CvBridge instance to convert between ROS image messages and OpenCV images.
        self.bridge = CvBridge()
        
        # Create a Pose2D message to store the detected ArUco marker's pose.
        self.msg_1 = Pose2D()
        self.msg_2 =Pose2D()
        self.msg_3 =Pose2D()
        
        self.i_0=0
        self.i_1=0
        self.i_2=0
        
        self.msg=[self.msg_1,self.msg_2,self.msg_3]
        self.topleft=None
        self.topright=None
        self.bottomleft=None
        self.bottomright=None
        self.caliberated_image=None
        self.arena_corners=[]
        bot_ids=[1,2,3]
        # self.bot_paths_1=[[] for _ in range(len(bot_ids))]
        self.bot_path_1 = []
        self.bot_path_2 = []
        self.bot_path_3 = []
        self.bot_paths=[self.bot_path_1,self.bot_path_2,self.bot_path_3]
        self.distorted_image = None
        self.transformed_image = None
        
        self.corners=[]
        self.ids=[]
        self.previous_tL=None
        self.previous_tR=None
        self.previous_bR=None
        self.previous_bL=None
        
        
    def image_callback(self,msg):
        # Convert the ROS image message to an OpenCV image.
        self.distorted_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")    
        
    # def trajectory(self,centre_x,centre_y):
    def iter_callback_1(self,msg:Int32):
        self.i_0=msg.data
       
    def iter_callback_2(self,msg:Int32):
        self.i_1=msg.data
       
    def iter_callback_3(self,msg:Int32):
        self.i_2=msg.data   
        

        
 
def main(args=None):
    rclpy.init(args=args)
    fb=FeedBack()
    rclpy.spin_once(fb)
    # transformed_image=fb.image_converter(fb.distorted_image)
    while rclpy.ok():
       
                # print(f"len_of_corners={len(corners)}")
                # print(f"corners={corners}")
                
                bot_ids=[1,2,3]
                fb.caliberated_image=camera_caliberation(fb.distorted_image)

                fb.corners,fb.ids=aruco_detect(fb.caliberated_image)
                fb.topleft,fb.topright,fb.bottomleft,fb.bottomright=get_corners(fb.corners,fb.ids)
                if fb.topleft is not None and fb.topright is not None and fb.bottomleft is not None and fb.bottomright is not None:
                    fb.arena_corners=np.float32([fb.topleft,fb.topright,fb.bottomleft,fb.bottomright])
                    fb.previous_bL=fb.bottomleft
                    fb.previous_bR=fb.bottomright
                    fb.previous_tL=fb.topleft
                    fb.previous_tR=fb.topright
                    
                    
                if fb.bottomleft is None:
                    fb.bottomleft=fb.previous_bL                   
                if fb.topright is None:
                    fb.topright=fb.previous_tR
                if fb.bottomright is None:
                    fb.bottomright=fb.previous_bR
                if fb.topleft is None:
                    fb.topleft=fb.previous_tL
                if fb.topleft is not None and fb.topright is not None and fb.bottomleft is not None and fb.bottomright is not None:

                    fb.transformed_image=perspective_transform(fb.caliberated_image,fb.arena_corners)

                
                # Check if marker ID 1 is detected.
                if fb.transformed_image is not None:
                    corners,ids=aruco_detect(thresholding(fb.transformed_image))
                
                    for i in range(len(corners)):
                        for j in range(len(bot_ids)):
                            if ids[i][0]==bot_ids[j]:
                                # print(ids,corners)
                                arr=corners[i][0]
                                # Calculate the center coordinates and orientation (theta) of the detected marker
                                centre_x_opencv = sum(arr[:, 0]) / 4
                                centre_y_opencv = sum(arr[:, 1]) / 4
                                # cv2.circle(fb.transformed_image, (int(centre_x_opencv), int(centre_y_opencv)), 5, (0, 255, 0), -1)
                                if j == 0:
                                    if fb.i_0 > 0:
                                        fb.bot_path_1.append((centre_x_opencv, centre_y_opencv))
                                elif j == 1:
                                    if fb.i_1 > 0:
                                        fb.bot_path_2.append((centre_x_opencv, centre_y_opencv))
                                elif j == 2:
                                    if fb.i_2 > 0:
                                        fb.bot_path_3.append((centre_x_opencv, centre_y_opencv))
                                
                                
                                text = f"ID: {ids[i][0]}"
                                cv2.putText(fb.transformed_image, text, (int(centre_x_opencv) - 20, int(centre_y_opencv) - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
                                
                                # for k in range(len(fb.bot_paths[j])):
                                for k in range(len(fb.bot_path_1)) :  
                                    point= fb.bot_path_1[k]
                                    if j==0:
                                        if fb.i_0>0:
                                            cv2.circle(fb.transformed_image,(int(point[0]),int(point[1])) ,1,(0,0,255),-1)
                                
                                for k in range(len(fb.bot_path_2)) : 
                                    point= fb.bot_path_2[k]
                                    if j==1:
                                        if fb.i_1>0:
                                
                                            cv2.circle(fb.transformed_image,(int(point[0]),int(point[1])) ,1,(0,255,0),-1)
                                
                                for k in range(len(fb.bot_path_3)) : 
                                    point= fb.bot_path_3[k]
                                    if j==2:
                                        
                                        if fb.i_2>0:
                                            cv2.circle(fb.transformed_image,(int(point[0]),int(point[1])) ,1,(255,0,0),-1)
                                    # convert the coordinates into cartesian plane
                                centre_x = centre_x_opencv
                                centre_y = 500-centre_y_opencv

                                # Calculate the orientation angle (theta) of the marker.
                                right_centre_x = (float((arr[1, 0] + arr[2, 0]) / 2))
                                right_centre_y = 500-float((arr[1, 1] + arr[2, 1]) / 2)            
                                theta = math.atan2((centre_y - right_centre_y) , centre_x - right_centre_x)
                                
                                theta+= math.pi
                                print(bot_ids[j],centre_x, centre_y, theta)
                                fb.msg[j].x=centre_x
                                fb.msg[j].y=centre_y
                                fb.msg[j].theta=theta
                                fb.pub1.publish(fb.msg[0])
                                fb.pub2.publish(fb.msg[1])
                                fb.pub3.publish(fb.msg[2])    

                                cv2.imshow('frame',fb.transformed_image)
                                cv2.waitKey(1)
  
                # cv2.waitKey(1)
                rclpy.spin_once(fb)
        
    fb.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
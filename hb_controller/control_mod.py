#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ 1545 ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
# from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose2D, Wrench
from my_robot_interfaces.msg import Goal   
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from control_utils import *
# import matplotlib.pyplot as plt          


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
    
        # Initialise the required variables
        # self.bot_1_x = [200,225,250,275,300,325,350]
        # self.bot_1_y = [150,150,150,150,150,150,150]
        self.bot_1_theta = 0.0
        # self.bot_1_x=[250]
        # self.bot_1_y=[250]
        
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        
        # Initialze Publisher and Subscriber
        self.lw_pub = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.rw_pub = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.fw_pub = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        self.vel_pub= self.create_publisher(Twist,'/cmd_vel/bot1',10)


        # self.pen_mode= self.create_publisher(Int32,'/pen_mode',10)
        self.pen_bool = self.create_publisher(Bool, 'pen1_down', 10)

        
        self.pose_subs = self.create_subscription(Pose2D, '/pen1_pose', self.odometryCb, 10)
        
        # self.kp = 0.4  # Proportional gain for position control
        self.ka = 0.04  # Proportional gain for angular control
        # dictionary for pid constants
        self.pid_const_linear={'Kp':0.111,'Ki':0.00,'Kd':0.0}
        self.pid_const_angular={'Kp':5,'Ki':0.1,'Kd':0.0}
        self.intg_const={'linear':0.0,'angular':0.0}
        self.last_error_const={'linear':0.0,'angular':0.0}
        self.i=0
        
        self.vel=Twist()
        
        # self.pen_mode_msg=Int32()
        self.pen_bool = Bool()

        self.rw_msg = Wrench()
        self.lw_msg = Wrench()
        self.fw_msg = Wrench()
        
        # self.msg_x=
        # self.msg_y=
        self.bot_1_x = [200,400,400,200,200]
        self.bot_1_y = [300,300,400,400,300]
        # self.bot_1_theta = 0.0

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        ) 

        # self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    # def goalCallBack(self, msg):     
        
        
    def odometryCb(self, pose: Pose2D):
        # Callback function to update robot's position and orientation from odometry data
        
        self.hb_x = pose.x
        self.hb_y = pose.y
        self.hb_theta = pose.theta
        
    
    
    def goalCallBack(self, msg):
        self.bot_1_x = msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta = msg.theta
        
        
    def pid(self,error, const, intg, last_error):
        prop = error
        intg = error + intg
        diff = error - last_error
        # compute balance
        balance = const['Kp'] * prop + const['Ki'] * intg + const['Kd'] * diff
        # update the last error
        last_error = error

        return balance


    def getAngVel(self,error, const, threshold_angle):
            ang_vel=0

            # if angular error more than the threshold then computer velocity
            if abs(error) > threshold_angle:
                if error > 3.14:
                    ang_vel = self.pid((error-6.28), const, self.intg_const['angular'], self.last_error_const['angular']) # from intg_params and last_error_params choose the intg and last_param 
                elif error < -3.14:                                                                         # meant for angular pid(w)
                    ang_vel = self.pid((error+6.28), const, self.intg_const['angular'], self.last_error_const['angular'])
                else:
                    ang_vel = self.pid(error, const, self.intg_const['angular'], self.last_error_const['angular'])

            return ang_vel
        
    def getangle(self,theta):
        if theta>3.14:
            theta=theta-2*math.pi
        elif theta<-3.14:
            theta=theta+2*math.pi
        return theta
         

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    # fig, ax = plt.subplots()
       
    # Main loop
    rclpy.spin_once(hb_controller)
    while rclpy.ok():
        if hb_controller.i < len(hb_controller.bot_1_x):
            x_goal=hb_controller.bot_1_x[hb_controller.i]
            y_goal=hb_controller.bot_1_y[hb_controller.i]
            theta_goal=0.0
                     
            # ax.plot(hb_controller.path_x, hb_controller.path_y, label='Bot 1 Path', color='blue')
            # ax.legend()
            # plt.pause(0.01)

            # print(f"x_g={x_goal}, y_g={y_goal},Q={theta_goal} at i={hb_controller.i}")
            
            # print(f"x={hb_x}, y={hb_y}, at_q={hb_theta}")
            
            print(hb_controller.hb_x)
            e_x = (x_goal - hb_controller.hb_x)
            e_y = (y_goal - hb_controller.hb_y)
            e_theta = (theta_goal - hb_controller.hb_theta)
            # Calculate desired velocity and angular velocity
            
                       
            distance_error = math.sqrt(math.pow((x_goal - hb_controller.hb_x), 2) + math.pow((y_goal - hb_controller.hb_y), 2))
            tolerance_dist = 6
            tolerance_theta = 0.5
            
            e_theta_rframe = e_theta
            e_x_rframe = (math.cos(hb_controller.hb_theta)) * (e_x) + (math.sin(hb_controller.hb_theta)) * (e_y)
            e_y_rframe = -(math.sin(hb_controller.hb_theta)) * (e_x) + (math.cos(hb_controller.hb_theta)) * (e_y)
            # Calculate the required velocity of bot for the next iteration(s)
            vel_x = hb_controller.pid(e_x_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
              
            
            vel_y = hb_controller.pid(e_y_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
            
            # vel_w = hb_controller.getAngVel(e_theta, hb_controller.ka, tolerance_theta)
            
            vel_w = hb_controller.getAngVel(e_theta_rframe,hb_controller.pid_const_angular,0.5)
            
            print(f"e_x={e_x_rframe},e_y={e_y_rframe},e_theta={hb_controller.getangle(e_theta)},at goal{x_goal},{y_goal}")
            
            # print(f"{hb_controller.hb_x},{hb_controller.hb_y},at goal{x_goal},{y_goal}")
                
            # print(f"e_x_rframe={e_x_rframe},e_y_rframe={e_y_rframe},e_theta={e_theta_rframe},at goal{x_goal},{y_goal}")
            # if abs(distance_error)<=tolerance_dist and hb_controller.i==0:
                
            #     hb_controller.pen_mode_msg=True
            #     hb_controller.pen_mode.publish(hb_controller.pen_mode_msg)
            #     time.sleep(0.5)
            if (abs(e_x_rframe))> tolerance_dist or (abs(e_y_rframe))>tolerance_dist or (abs(hb_controller.getangle(e_theta)))>0.5:
                fw_vel_x, rw_vel_x, lw_vel_x = inverse_kinematics(vel_x, vel_y, vel_w)
                
                
                fw_vel_x, rw_vel_x, lw_vel_x =Vel2RPM(fw_vel_x,rw_vel_x,lw_vel_x)
                print(f'rw_vel_x ={rw_vel_x}')
                # fw_vel_x, rw_vel_x, lw_vel_x=hb_controller.smallRPM(1.2,fw_vel_x,rw_vel_x,lw_vel_x)
                fw_vel_x, rw_vel_x, lw_vel_x=clip_wheel_vel(fw_vel_x,rw_vel_x,lw_vel_x)
                if fw_vel_x<-5:
                    fw_vel_x=map_vel(fw_vel_x,-40,-5,-40,-11)
                elif fw_vel_x>5:
                    fw_vel_x=map_vel(fw_vel_x,5,40,11,40)
                if rw_vel_x<-3:
                    rw_vel_x=map_vel(rw_vel_x,-40,-3,-40,-11)
                elif rw_vel_x>3:
                    rw_vel_x=map_vel(rw_vel_x,3,40,11,40)
                
                if lw_vel_x<-5:
                    lw_vel_x=map_vel(lw_vel_x,-40,-5,-40,-11)
                elif lw_vel_x>5:
                    lw_vel_x=map_vel(lw_vel_x,5,40,11,40)
                
                
                # max_=max(abs(fw_vel_x),abs(rw_vel_x),abs(lw_vel_x))
                # min_=min(abs(fw_vel_x),abs(rw_vel_x),abs(lw_vel_x))
                # fw_vel_x= hb_controller.map_vel(fw_vel_x,min_,max_,15,50)
                # rw_vel_x= hb_controller.map_vel(rw_vel_x,min_,max_,15,50)
                # lw_vel_x= hb_controller.map_vel(lw_vel_x,min_,max_,15,50)
                
                
                
                
                # hb_controller.fw_msg.force.y = fw_vel_x
                # hb_controller.rw_msg.force.y = rw_vel_x
                # hb_controller.lw_msg.force.y = lw_vel_x
                # print(f'fw_vel_x {fw_vel_x}',f'lw_vel_x {lw_vel_x}',f'rw_vel_x {rw_vel_x}')
                
                # print(f"v={fw_vel_x},{rw_vel_x},{lw_vel_x} at e_err={e_theta}")
                # hb_controller.fw_pub.publish(hb_controller.fw_msg)
                # hb_controller.lw_pub.publish(hb_controller.lw_msg) 
                # hb_controller.rw_pub.publish(hb_controller.rw_msg)
                hb_controller.vel.linear.x=fw_vel_x
                hb_controller.vel.linear.y=lw_vel_x
                hb_controller.vel.linear.z=rw_vel_x
                hb_controller.vel_pub.publish(hb_controller.vel)
                print("running")
                
                # if(hb_controller.i>0 and hb_controller.i<len((hb_controller.bot_1_x))):
                #     hb_controller.pen_mode_msg=1
                #     hb_controller.pen_mode.publish(hb_controller.pen_mode_msg)

            elif (abs(e_x_rframe))< tolerance_dist and (abs(e_y_rframe))<tolerance_dist and ((abs(e_theta)))<0.5:
                
                # Stop the robot by setting wheel forces to zero if goal is reached
             
                # hb_controller.vel.linear.x = 0.0
                # hb_controller.vel.linear.y = 0.0
                # hb_controller.vel.linear.z = 0.0
                # hb_controller.vel_pub.publish(hb_controller.vel)
                    
                
                
                # # print(f"i={hb_controller.i}")
                # hb_controller.fw_pub.publish(hb_controller.fw_msg)
                # hb_controller.lw_pub.publish(hb_controller.lw_msg)
                # hb_controller.rw_pub.publish(hb_controller.rw_msg)
                # hb_controller.vel.linear.x=fw_vel_x
                # hb_controller.vel.linear.y= rw_vel_x
                # hb_controller.vel.linear.z= lw_vel_x
                
                print("zero")
                # time.sleep(1)
                
                hb_controller.i+=1
                if hb_controller.i==len(hb_controller.bot_1_x):
                    hb_controller.vel.linear.x = 0.0
                    hb_controller.vel.linear.y = 0.0
                    hb_controller.vel.linear.z=0.0
                    hb_controller.vel_pub.publish(hb_controller.vel)
                    
                    print("I QUIT ;)")
                    
                    
            
                # if(hb_controller.i==len(hb_controller.bot_1_x)-1):
                #     hb_controller.pen_bool.publish(False)
                #     time.sleep(0.5)
                   
         
            # publishing to twist
            
            

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()

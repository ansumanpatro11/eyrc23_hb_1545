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


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
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

# import matplotlib.pyplot as plt          


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        

        # Initialise the required variables
        # self.bot_1_x = []
        # self.bot_1_y = []
        # self.bot_1_theta = 0.0
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        
        # Initialze Publisher and Subscriber
        self.lw_pub = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.rw_pub = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.fw_pub = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        self.vel_pub= self.create_publisher(Twist,'/cmd_vel/bot1',10)


        self.pen_mode= self.create_publisher(Int32,'/pen_mode',10)
        self.pen_bool = self.create_publisher(Bool, '\pen1_down', 10)

        
        self.pose_subs = self.create_subscription(Pose2D, '/pen1_pose', self.odometryCb, 10)
        
        self.kp = 24  # Proportional gain for position control
        self.ka = 28  # Proportional gain for angular control
        
        self.i=0
        
        self.vel=Twist()
        
        self.pen_mode_msg=Int32()
        self.pen_bool = Bool()

        self.rw_msg = Wrench()
        self.lw_msg = Wrench()
        self.fw_msg = Wrench()
        
        # self.msg_x=
        # self.msg_y=
        self.bot_1_x = [200,175,125,100,125,175,200]
        self.bot_1_y = [150,200,200,150,100,100,150]
        self.bot_1_theta = 0.0



        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        # self.subscription = self.create_subscription(
        #     Goal,  
        #     'hb_bot_1/goal',  
        #     self.goalCallBack,  # Callback function to handle received messages
        #     10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        # ) 

        # self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    def inverse_kinematics(self, vx, vy, w):
        # Calculate wheel forces based on desired velocity and angular velocity
        u1=0.33*w+0.67*vx
        u2=0.33*w-0.33*vx+0.58*vy
        u3=0.33*w-0.33*vx-0.58*vy

        return u1, u2, u3

    # def goalCallBack(self, msg):
        
        
        
    def odometryCb(self, pose: Pose2D):
        # Callback function to update robot's position and orientation from odometry data
        
        self.hb_x = pose.x
        self.hb_y = pose.y
        self.hb_theta = pose.theta
        
    # def getError(self, error):



    #         if error > 3.14:
    #             ang_error = (error-6.28)
    #         elif error < -3.14:
    #             ang_error = (error+6.28)
    #         else:
    #             ang_error = error
             
    #         return ang_error
    

    def theta_modify(self,g_theta):
        
        
        if g_theta>3.14:
            g_theta=g_theta-math.pi
            
        else:
            g_theta=g_theta

        return g_theta        

    
    def limitVel(self,vel):
        
        if vel>0:
            get_vel=min(120,vel)

        else:
            get_vel=max(-120,vel)   
            
        return get_vel  
    
    def clip_wheel_vel(self,fw_vel,rw_vel,lw_vel):
        max_vel=max(abs(lw_vel),abs(rw_vel),abs(fw_vel))
        
        if abs(fw_vel) > 90 or abs(rw_vel) >90 or abs(lw_vel)>90  :
            fw_vel=(fw_vel/max_vel)*120
            lw_vel=(lw_vel/max_vel)*120
            rw_vel=(rw_vel/max_vel)*120
        
        return fw_vel,rw_vel,lw_vel 
    

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    # fig, ax = plt.subplots()
       
    # Main loop
    while rclpy.ok():
        if hb_controller.i < len(hb_controller.bot_1_x):
            x_goal=hb_controller.bot_1_x[hb_controller.i]
            y_goal=hb_controller.bot_1_y[hb_controller.i]
            theta_goal=hb_controller.bot_1_theta
            
            
            
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
            tolerance_dist = 1
            tolerance_theta = 0.55
            
            e_theta_rframe = e_theta
            e_x_rframe = (math.cos(hb_controller.hb_theta)) * (e_x) + (math.sin(hb_controller.hb_theta)) * (e_y)
            e_y_rframe = -(math.sin(hb_controller.hb_theta)) * (e_x) + (math.cos(hb_controller.hb_theta)) * (e_y)
            # Calculate the required velocity of bot for the next iteration(s)
            vel_x = e_x_rframe * hb_controller.kp
              
            
            vel_y =e_y_rframe * hb_controller.kp
            
            
            # vel_w = hb_controller.getAngVel(e_theta, hb_controller.ka, tolerance_theta)
            
            vel_w = e_theta*hb_controller.ka
            
            # print(f"vel={vel_x},{vel_y},{vel_w}")
                
            
            if abs(distance_error)>= tolerance_dist :
                fw_vel_x, rw_vel_x, lw_vel_x = hb_controller.inverse_kinematics(vel_x, vel_y, vel_w)
                fw_vel_x, rw_vel_x, lw_vel_x =hb_controller.clip_wheel_vel(fw_vel_x,rw_vel_x,lw_vel_x)
                hb_controller.fw_msg.force.y = fw_vel_x
                hb_controller.rw_msg.force.y = rw_vel_x
                hb_controller.lw_msg.force.y = lw_vel_x
                print(f'fw_vel_x {fw_vel_x}',f'lw_vel_x {lw_vel_x}',f'rw_vel_x {rw_vel_x}')
                
                # print(f"v={fw_vel_x},{rw_vel_x},{lw_vel_x} at e_err={e_theta}")
                hb_controller.fw_pub.publish(hb_controller.fw_msg)
                hb_controller.lw_pub.publish(hb_controller.lw_msg)
                hb_controller.rw_pub.publish(hb_controller.rw_msg)
                hb_controller.vel.linear.x=fw_vel_x
                hb_controller.vel.linear.y= rw_vel_x
                hb_controller.vel.linear.z= lw_vel_x
                hb_controller.vel_pub.publish(hb_controller.vel)
                
                # if(hb_controller.i>0 and hb_controller.i<len((hb_controller.bot_1_x))):
                #     hb_controller.pen_mode_msg=1
                #     hb_controller.pen_mode.publish(hb_controller.pen_mode_msg)
                
                
                    
            if abs(distance_error)<=tolerance_dist and hb_controller.i==0:
                
                hb_controller.pen_mode_msg=True
                hb_controller.pen_mode.publish(hb_controller.pen_mode_msg)
                time.sleep(0.5)
                





            else:
                # Stop the robot by setting wheel forces to zero if goal is reached
                hb_controller.fw_msg.force.y = 0.0
                hb_controller.rw_msg.force.y = 0.0
                hb_controller.lw_msg.force.y = 0.0
                # print("zero")
                # print(f"i={hb_controller.i}")
                hb_controller.fw_pub.publish(hb_controller.fw_msg)
                hb_controller.lw_pub.publish(hb_controller.lw_msg)
                hb_controller.rw_pub.publish(hb_controller.rw_msg)
                hb_controller.vel.linear.x=fw_vel_x
                hb_controller.vel.linear.y= rw_vel_x
                hb_controller.vel.linear.z= lw_vel_x
                hb_controller.vel_pub.publish(hb_controller.vel)
                
                hb_controller.i+=1
                if(hb_controller.i==len(hb_controller.bot_1_x))-1:
                    hb_controller.pen_bool.publish(False)
                    time.sleep(0.5)
                   
                
            
            
            # publishing to twist
            
            

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()

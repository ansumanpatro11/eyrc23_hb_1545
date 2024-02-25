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
# Author List:		[Ansuman,Amit,Abhishek ]
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist, Pose2D, Wrench
from my_robot_interfaces.msg import Goal   
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from control_utils import *
from goals import *
from cont_goals import *



class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_2')
    
      
        self.bot_2_x=[]
        self.bot_2_y=[]
        self.prev_time = time.time()

        
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        
        # Initialze Publisher and Subscriber
        self.lw_pub = self.create_publisher(Wrench, '/hb_bot_2/left_wheel_force', 10)
        self.rw_pub = self.create_publisher(Wrench, '/hb_bot_2/right_wheel_force', 10)
        self.fw_pub = self.create_publisher(Wrench, '/hb_bot_2/rear_wheel_force', 10)

        self.vel_pub= self.create_publisher(Twist,'/cmd_vel/bot2',1)
        self.iter_pub = self.create_publisher(Int32, '/i_2', 1)


        self.pen_bool = self.create_publisher(Bool, '/pen2_down', 1)

        
        self.pose_subs = self.create_subscription(Pose2D, '/pen2_pose', self.odometryCb, 1)
        
        
        
        self.pid_const_linear={'Kp':0.69,'Ki':0.00,'Kd':0.00000001}
        self.pid_const_angular={'Kp':3.6,'Ki':0,'Kd':0.00001} #without using time at 0.2 theta
        self.intg_const={'linear':0.0,'angular':0.0}
        self.last_error_const={'linear':0.0,'angular':0.0}
        self.i=0
        self.ctr=0
        self.iter=Int32()
        self.vel=Twist()
        
        
        self.pen_bool_msg = Bool()

        self.rw_msg = Wrench()
        self.lw_msg = Wrench()
        self.fw_msg = Wrench()
        
        
        detected_points_file = "/home/ansuman/eyrc_HB/eyrc23_hb_1545/hb_controller/HB_1545_task5/codes/controller/detected_points.txt"

        self.bot_2_x,self.bot_2_y=bot_2_goals(self.bot_2_x,self.bot_2_y)
        
        self.bot_2_theta = 0.0

        # self.x_dict,self.y_dict=read_detected_points(detected_points_file)
        # if self.ctr==0:
        #     self.bot_2_x=self.x_dict['contour24']
        #     self.bot_2_y=self.y_dict['contour24']
        


        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    # def goalCallBack(self, msg):     
        
        
    def odometryCb(self, pose: Pose2D):
        # Callback function to update robot's position and orientation from odometry data
        
        self.hb_x = pose.x
        self.hb_y = pose.y
        self.hb_theta = pose.theta
        
    
    
    def goalCallBack(self, msg):
        self.bot_2_x = msg.x
        self.bot_2_y = msg.y
        self.bot_2_theta = msg.theta
        
        
    def pid(self,error, const, intg, last_error):
        self.current_time = time.time()

        dt = self.current_time - self.prev_time

        prop = error
        intg = error + intg
        diff = error - last_error
        # compute balance
        balance = const['Kp'] * prop + const['Ki'] * intg*dt + const['Kd'] * diff/dt
        # update the last error
        last_error = error
        self.prev_time = self.current_time
 
        return balance
    def fw_pwm(self,rpm):
        pwm=0
     
        if rpm>10:
            d={'a':0.0002,'b':-0.0185,'c':-1.3463,'d':103.2878}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0001,'b':-0.0152,'c':-2.2642,'d':79.1319}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        else:
            pwm=90.0
        return pwm
    
    def lw_pwm(self,rpm):
        pwm=0
     
        if rpm>10:
            d={'a':-0.0004,'b':0.0312,'c':-2.4746,'d':109.1201}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0003,'b':-0.0287,'c':-2.6438,'d':75.6338}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        else:
            pwm=90.0
        return pwm
          
    def rw_pwm(self,rpm):
        pwm=0
     
        if rpm>10:
            d={'a':-0.0012,'b':0.0846,'c':-3.5111,'d':113.3038}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0002,'b':-0.0233,'c':-2.5526,'d':77.9788}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        else:
            pwm=90.0
        return pwm

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
        if hb_controller.i < len(hb_controller.bot_2_x):
            x_goal=hb_controller.bot_2_x[hb_controller.i]
            y_goal=hb_controller.bot_2_y[hb_controller.i]
            theta_goal=0.0
                     
            
            
            print(hb_controller.hb_x)
            e_x = (x_goal - hb_controller.hb_x)
            e_y = (y_goal - hb_controller.hb_y)
            e_theta = (theta_goal - hb_controller.hb_theta)
            # Calculate desired velocity and angular velocity
            
                       
            distance_error = math.sqrt(math.pow((x_goal - hb_controller.hb_x), 2) + math.pow((y_goal - hb_controller.hb_y), 2))
            tolerance_dist = 6.5
            tolerance_theta = 0.35
            
            e_theta_rframe = e_theta
            e_x_rframe = (math.cos(hb_controller.hb_theta)) * (e_x) + (math.sin(hb_controller.hb_theta)) * (e_y)
            e_y_rframe = -(math.sin(hb_controller.hb_theta)) * (e_x) + (math.cos(hb_controller.hb_theta)) * (e_y)
            # Calculate the required velocity of bot for the next iteration(s)
            vel_x = hb_controller.pid(e_x_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
              
            
            vel_y = hb_controller.pid(e_y_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
            
            # vel_w = hb_controller.getAngVel(e_theta, hb_controller.ka, tolerance_theta)
            
            vel_w = hb_controller.getAngVel(e_theta_rframe,hb_controller.pid_const_angular,tolerance_theta)
            
            print(f"e_x={e_x_rframe},e_y={e_y_rframe},e_theta={hb_controller.getangle(e_theta)},at goal{x_goal},{y_goal}")
            
            
            # if (abs(e_x_rframe))< tolerance_dist and (abs(e_y_rframe))<tolerance_dist and (abs(hb_controller.getangle(e_theta)))<tolerance_theta and hb_controller.i==0:
                
            #     hb_controller.pen_bool_msg.data=True
            #     hb_controller.pen_bool.publish(hb_controller.pen_bool_msg)
            #     time.sleep(2)
            if (abs(e_x_rframe))> tolerance_dist or (abs(e_y_rframe))>tolerance_dist or (abs(hb_controller.getangle(e_theta)))>tolerance_theta:
                fw_vel_x, rw_vel_x, lw_vel_x = inverse_kinematics(vel_x, vel_y, vel_w)
                
                
                fw_vel_x, rw_vel_x, lw_vel_x =Vel2RPM(fw_vel_x,rw_vel_x,lw_vel_x)
                print(f'fw_vel_x={fw_vel_x},lw_vel_x={lw_vel_x}rw_vel_x ={rw_vel_x}')
                # fw_vel_x, rw_vel_x, lw_vel_x=hb_controller.smallRPM(1.2,fw_vel_x,rw_vel_x,lw_vel_x)
                fw_vel_x, rw_vel_x, lw_vel_x=clip_wheel_vel(fw_vel_x,rw_vel_x,lw_vel_x)
                if fw_vel_x<-3:
                    fw_vel_x=map_vel(fw_vel_x,-40,-3,-40,-11)
                elif fw_vel_x>3:
                    fw_vel_x=map_vel(fw_vel_x,3,40,11,40)
                if rw_vel_x<-3:
                    rw_vel_x=map_vel(rw_vel_x,-40,-3,-40,-11)
                elif rw_vel_x>3:
                    rw_vel_x=map_vel(rw_vel_x,3,40,11,40)
                if lw_vel_x<-3:
                    lw_vel_x=map_vel(lw_vel_x,-40,-3,-40,-11)
                elif lw_vel_x>3:
                    lw_vel_x=map_vel(lw_vel_x,3,40,11,40)
                fw_vel_x=hb_controller.fw_pwm(fw_vel_x)
                lw_vel_x=hb_controller.lw_pwm(lw_vel_x)
                rw_vel_x=hb_controller.rw_pwm(rw_vel_x)
         

                
                
                print(f"v={fw_vel_x},{rw_vel_x},{lw_vel_x} at e_err={e_theta}")
            
                
                hb_controller.vel.linear.x=float(fw_vel_x)
                hb_controller.vel.linear.y=float(lw_vel_x)
                hb_controller.vel.linear.z=float(rw_vel_x)
                hb_controller.vel_pub.publish(hb_controller.vel)
                print("running")
                
                

            elif (abs(e_x_rframe))< tolerance_dist and (abs(e_y_rframe))<tolerance_dist and (abs(hb_controller.getangle(e_theta)))<tolerance_theta:
                
                # Stop the robot by setting wheel forces to zero if goal is reached
             
                hb_controller.vel.linear.x = 90.0
                hb_controller.vel.linear.y = 90.0
                hb_controller.vel.linear.z = 90.0
                
         
                hb_controller.vel_pub.publish(hb_controller.vel)
 
                print("zero")
                # time.sleep(1)
                
                hb_controller.i+=1
                hb_controller.iter.data=hb_controller.i
                hb_controller.iter_pub.publish(hb_controller.iter)
                if hb_controller.i==len(hb_controller.bot_2_x):
                    # time.sleep(2)
                    hb_controller.vel.linear.x = 90.0
                    hb_controller.vel.linear.y = 90.0
                    hb_controller.vel.linear.z=90.0  
                
                    hb_controller.vel_pub.publish(hb_controller.vel)
                    hb_controller.pen_bool_msg.data=False
                    hb_controller.pen_bool.publish(hb_controller.pen_bool_msg)
                    
                    hb_controller.ctr+=1
                    time.sleep(0.5)
                    if hb_controller.ctr==1:
                        hb_controller.bot_2_x=hb_controller.x_dict['contour20']
                        hb_controller.bot_2_y=hb_controller.y_dict['contour20']
                        hb_controller.i=0
                        
                    elif hb_controller.ctr==2:
                        hb_controller.bot_2_x=hb_controller.x_dict['contour10']
                        hb_controller.bot_2_y=hb_controller.y_dict['contour10']
                        hb_controller.i=0
                    elif hb_controller.ctr==3:
                        hb_controller.bot_2_x=hb_controller.x_dict['contour7']
                        hb_controller.bot_2_y=hb_controller.y_dict['contour7']
                        hb_controller.i=0
                        print("I DID THE JOB ;)")
                        
                    else:
                        hb_controller.vel.linear.x = 90.0
                        hb_controller.vel.linear.y = 90.0
                        hb_controller.vel.linear.z=90.0  
                                
                        hb_controller.vel_pub.publish(hb_controller.vel)
                        time.sleep(500)
                        
                    
                    
            
                # if(hb_controller.i==len(hb_controller.bot_2_x)):
                #     hb_controller.pen_bool_msg.data=False
                #     hb_controller.pen_bool.publish(hb_controller.pen_bool_msg)
                #     time.sleep(1)
                   
         
            # publishing to twist
            
            

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()

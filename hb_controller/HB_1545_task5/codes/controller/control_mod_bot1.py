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
# Author List:		[ Ansuman, Amit, Abhishek ]
# Filename:		controller_bot1.py
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
from std_srvs.srv import Empty
from contours import *
from contour_goals import *
from cont_goals import *







class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_1')
        
        self.bot_1_x=[]
        self.bot_1_y=[]
        
        self.hb_x=0.0
        self.hb_y=0.0
        self.hb_theta=0.0
        self.prev_time = time.time()

        
        # Initialze Publisher and Subscriber

        self.vel_pub= self.create_publisher(Twist,'/cmd_vel/bot1',1)
        self.pen_bool = self.create_publisher(Bool, '/pen1_down_intermediate', 1)
        self.pen_final=self.create_publisher(Bool,'/pen1_down',1)
        self.pen_sub=self.create_subscription(Bool,'/all_down',self.pen_callback)

        
        
        
        


        self.pen_bool = self.create_publisher(Bool, '/pen1_down', 1)
        self.iter_pub = self.create_publisher(Int32, '/i_1', 1)


        
        self.pose_subs = self.create_subscription(Pose2D, '/pen1_pose', self.odometryCb, 1)
        
    
        
        self.pid_const_linear={'Kp':0.6,'Ki':0.0,'Kd':0.0000001}
        self.pid_const_angular={'Kp':2.25,'Ki':0.0,'Kd':0.00007}
        self.intg_const={'linear':0.0,'angular':0.0}
        self.last_error_const={'linear':0.0,'angular':0.0}
        self.i=0
        self.iter=Int32()
        self.vel=Twist()
        
        self.pen_bool_msg = Bool()
        self.pen_final_msg=Bool()
        

        self.rw_msg = Wrench()
        self.lw_msg = Wrench()
        self.fw_msg = Wrench()
        
        detected_points_file = "/home/ansuman/eyrc_HB/eyrc23_hb_1545/hb_controller/HB_1545_task5/codes/controller/detected_points.txt"

         
        
        # self.bot_1_x,self.bot_1_y=goals(self.bot_1_x,self.bot_1_y)
        image=cv2.imread('/home/ansuman/eyrc_HB/eyrc23_hb_1545/hb_controller/HB_1545_task5/codes/controller/starr.jpeg')
        # self.x_dict,self.y_dict=read_detected_points(detected_points_file)
        # self.bot_1_x=self.x_dict['contour25']
        # self.bot_1_y=self.y_dict['contour25']
        
        self.bot_1_x,self.bot_1_y=bot_1_goals(self.bot_1_x,self.bot_1_y)
        self.bot_1_theta = 0.0

        
        self.rate = self.create_rate(100)

        
    def pen_callback(self,msg:Bool):
        self.pen=msg.data    
        
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
        self.current_time = time.time()
    
    # Calculate time elapsed since last iteration
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
            d={'a':-0.0012,'b':0.0923,'c':-3.8639,'d':119.1027}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0006,'b':-0.0478,'c':-2.8861,'d':75.2306}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        else:
            pwm=90.0
        return pwm
    
    def lw_pwm(self,rpm):
        pwm=0
     
        if rpm>10:
            d={'a':-0.0010,'b':0.0837,'c':-3.7968,'d':119.365}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0007,'b':-0.0601,'c':-3.2047,'d':71.6505}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        else:
            pwm=90.0
        return pwm
          
    def rw_pwm(self,rpm):
        pwm=0
     
        if rpm>10:
            d={'a':0.0012,'b':-0.1196,'c':1.1861,'d':85.5759}
            pwm=int(d['a']* pow(rpm, 3) + d['b'] * pow(rpm, 2) + d['c'] * rpm + d['d'])
        elif rpm<-10:
            d={'a':-0.0002,'b':-0.0257,'c':-3.0805,'d':69.3717}
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
         
    def stop_flag_callback(self, request, response):
        print("Stop_Flag service invoked")
        response = Empty()
        return response     

def main(args=None):
    rclpy.init(args=args)
    # ff=feedback_for_fast()
    hb_controller = HBController()
    # fig, ax = plt.subplots()
       
    # Main loop
    rclpy.spin_once(hb_controller)
    
    while rclpy.ok():
        if hb_controller.i < len(hb_controller.bot_1_x):
            x_goal=hb_controller.bot_1_x[hb_controller.i]
            y_goal=500-hb_controller.bot_1_y[hb_controller.i]
            theta_goal=0.0
                     
            
            
            print(hb_controller.hb_x)
            e_x = (x_goal - hb_controller.hb_x)
            e_y = (y_goal - hb_controller.hb_y)
            e_theta = (theta_goal - hb_controller.hb_theta)
            # Calculate desired velocity and angular velocity
            
                       
            distance_error = math.sqrt(math.pow((x_goal - hb_controller.hb_x), 2) + math.pow((y_goal - hb_controller.hb_y), 2))
            tolerance_dist = 5
            tolerance_theta = 0.36
            
            e_theta_rframe = e_theta
            e_x_rframe = (math.cos(hb_controller.hb_theta)) * (e_x) + (math.sin(hb_controller.hb_theta)) * (e_y)
            e_y_rframe = -(math.sin(hb_controller.hb_theta)) * (e_x) + (math.cos(hb_controller.hb_theta)) * (e_y)
            # Calculate the required velocity of bot for the next iteration(s)
            vel_x = hb_controller.pid(e_x_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
              
            
            vel_y = hb_controller.pid(e_y_rframe,hb_controller.pid_const_linear,hb_controller.intg_const['linear'], hb_controller.last_error_const['linear'])
            
            # vel_w = hb_controller.getAngVel(e_theta, hb_controller.ka, tolerance_theta)
            
            vel_w = hb_controller.getAngVel(e_theta_rframe,hb_controller.pid_const_angular,tolerance_theta)
            
            print(f"e_x={e_x_rframe},e_y={e_y_rframe},e_theta={hb_controller.getangle(e_theta)},at goal{x_goal},{y_goal}")
            
            # print(f"{hb_controller.hb_x},{hb_controller.hb_y},at goal{x_goal},{y_goal}")
                
            if (abs(e_x_rframe))< tolerance_dist and (abs(e_y_rframe))<tolerance_dist and (abs(hb_controller.getangle(e_theta)))<0.55 and hb_controller.i==0:
                hb_controller.pen_bool_msg.data=True
                hb_controller.pen_bool.publish(hb_controller.pen_bool_msg)
                time.sleep(1)
            
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
                
                
                
                if hb_controller.i==0 and hb_controller.pen is True: 
                
                    hb_controller.pen_final_msg.data=True
                    hb_controller.pen_final.publish(hb_controller.pen_final_msg)
                
                    hb_controller.i+=1
                elif hb_controller.i==0 and hb_controller.pen is not True:
                    hb_controller.vel.linear.x = 0.0
                    hb_controller.vel.linear.y = 0.0
                    hb_controller.vel.linear.z=0.0  
                # time.sleep(1)
                
                hb_controller.i+=1
                hb_controller.iter.data=hb_controller.i
                hb_controller.iter_pub.publish(hb_controller.iter)
                if hb_controller.i==len(hb_controller.bot_1_x):
                    # time.sleep(2)
                    hb_controller.vel.linear.x = 90.0
                    hb_controller.vel.linear.y = 90.0
                    hb_controller.vel.linear.z= 90.0 
                                    
                    hb_controller.vel_pub.publish(hb_controller.vel)
                    # time.sleep(0.5)
                    print("I DID THE JOB ;)")
                    time.sleep(500)
                    
                    
            
                # if(hb_controller.i==60):
                #     hb_controller.pen_bool_msg.data=False
                #     hb_controller.pen_bool.publish(hb_controller.pen_bool_msg)
                #     stop_flag_service = hb_controller.create_service(Empty, 'Stop_Flag', hb_controller.stop_flag_callback)

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

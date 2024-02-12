import rclpy
from rclpy.node import Node
import time
import math
# from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose2D, Wrench
from my_robot_interfaces.msg import Goal   
from std_msgs.msg import Int32
from std_msgs.msg import Bool

def Vel2RPM(fw_vel,rw_vel, lw_vel):
       fw_rpm=fw_vel*30/math.pi
       lw_rpm=lw_vel*30/math.pi
       rw_rpm=rw_vel*30/math.pi
       
       return fw_rpm,rw_rpm,lw_rpm

def map_vel(vel, in_min,in_max, out_min,  out_max) :
        mapped=(vel - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

            
        return mapped
 
def smallRPM(self,fact,vel_fw,vel_rw,vel_lw):
        min_rpm=0
        min_rpm=min(vel_fw,vel_lw,vel_rw)
        
        if abs(vel_fw)<10 :
            vel_fw*=fact
        if abs(vel_lw)<10:
            vel_lw*=fact
        if abs(vel_rw)<10:
            vel_rw*=fact
        return vel_fw,vel_rw,vel_lw

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
 
 

        # if abs(fw_rpm)>50 or abs(lw_rpm)>50 or abs(rw_rpm)>50:
        #     max_rpm_magnitude=max(abs(fw_rpm), abs(lw_rpm), abs(rw_rpm))

        #     fw_rpm=float(fw_rpm)/max_rpm_magnitude*50
        #     lw_rpm=float(lw_rpm)/max_rpm_magnitude*50
        #     rw_rpm=float(rw_rpm)/max_rpm_magnitude*50
            
        # if abs(fw_rpm)>50 or abs(lw_rpm)>50 or abs(rw_rpm)>50:
            
        # return fw_rpm, lw_rpm, rw_rpm
        
def limitVel(self,vel):
        
        if vel>0:
            get_vel=min(80,vel)

        else:
            get_vel=max(-80,vel)   
            
        return get_vel  
 
# def clip_wheel_vel(self,fw_vel,rw_vel,lw_vel):
#         max_vel=max(abs(lw_vel),abs(rw_vel),abs(fw_vel))
        
#         if abs(fw_vel) > 80 or abs(rw_vel) >80 or abs(lw_vel)>80  :
#             fw_vel=(fw_vel/max_vel)*80
#             lw_vel=(lw_vel/max_vel)*80
#             rw_vel=(rw_vel/max_vel)*80
            
        
#         return fw_vel,rw_vel,lw_vel 
    
def getError(self, error):



            if error > 3.14:
                ang_error = (error-6.28)
            elif error < -3.14:
                ang_error = (error+6.28)
            else:
                ang_error = error
             
            return ang_error
     
def theta_modify(self,g_theta):
        
        
        if g_theta>3.14:
            g_theta=g_theta-math.pi
            
        else:
            g_theta=g_theta

        return g_theta    
 
 
def limitVel(self,vel):
        
        if vel>0:
            get_vel=min(80,vel)

        else:
            get_vel=max(-80,vel)   
            
        return get_vel  
    

    
def inverse_kinematics( vx, vy, w):
        # Calculate wheel forces based on desired velocity and angular velocity
        r=1.9
        u1=(0.33*w+0.67*vx)/r
        u2=(0.33*w-0.33*vx+0.58*vy)/r
        u3=(0.33*w-0.33*vx-0.58*vy)/r

        return u1, u2, u3
    
    
def clip_wheel_vel(fw_vel,rw_vel,lw_vel):
        max_vel=max(abs(lw_vel),abs(rw_vel),abs(fw_vel))
        
        if abs(fw_vel) > 40 or abs(rw_vel) >40 or abs(lw_vel)>40 :
            fw_vel=(fw_vel/max_vel)*40
            lw_vel=(lw_vel/max_vel)*40
            rw_vel=(rw_vel/max_vel)*40
            
        
        return fw_vel,rw_vel,lw_vel 
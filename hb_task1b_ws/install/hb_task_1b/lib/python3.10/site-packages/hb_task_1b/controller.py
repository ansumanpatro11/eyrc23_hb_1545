#!/usr/bin/env python3

########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1B ###########################################
# Team ID:1545
# Team Leader Name:Ansuman Patro
# Team Members Name:Amit Prasad Singh
#                   Abhisek Beuria
#                   Gampala Hema Sai Surya Teja
# College:National Institute of Technology, Rourkela
########################################################################################################################



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose,Pose2D
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
from functools import partial

class controller(Node):

    def __init__(self):
        super().__init__('controller')
        
        # Initialize Publisher and Subscriber
        self.pub1 = self.create_publisher(Twist, '/hb_bot_1/cmd_vel', 10)  # Create a publisher for robot velocity
        self.pose_subs = self.create_subscription(Pose2D, '/detected_aruco', self.odometryCb, 10)  # Create a subscriber for robot odometry
        
        # Initialize variables
        self.vel = Twist()  # Initialize a Twist message for controlling robot velocity
        self.kp_x = 3.12  # Proportional gain for x-axis
        self.kp_y = 3.12  # Proportional gain for y-axis
        self.kp_w = 7.2   # Proportional gain for angular velocity (yaw)
        self.index = 0    # Index for the goal pose
        self.flag = 0     # Flag to indicate the end of the goal list
        self.ang_thresh = float(math.pi) / 181  # Angular threshold for goal orientation
        
        # Initialize client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')
        self.request = NextGoal.Request() 
   
    def send_request(self, index):
        # Send a request to the "next_goal" service
        self.request.request_goal = index
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.future = self.cli.call_async(self.request)
    
    def odometryCb(self, msg):
        # Callback function to update robot's position and orientation from odometry data
        global hb_x, hb_y, hb_theta
        hb_x = msg.x
        hb_y = msg.y
        hb_theta = msg.theta
    
    def limit_vel_lin(self, applied_vel):
        # Limit linear velocity to a maximum value of 15
        vel = applied_vel
        if abs(applied_vel) > 15:
            if applied_vel > 0:
                vel = 15
            else:
                vel = -15
        return vel
    
    def limit_vel_ang(self, vel):
        # Limit angular velocity to a maximum value of 15
        angvel = vel
        if abs(vel) > 15:
            if vel > 0:
                angvel = 15
            else:
                angvel = -15
        return angvel

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the controller class
    ebot_controller = controller()

    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)
    
    # Main loop
    while rclpy.ok():
        if ebot_controller.future.done():
            try:
                
                # Response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().info('Service call failed %r' % (e,))
            else:

                
                x_goal = response.x_goal
                y_goal = response.y_goal
                theta_goal = response.theta_goal
                
                ebot_controller.flag = response.end_of_list
                # Extract goal pose information from the service response
                
                

                # Calculate error in global frame
                distance_error = math.sqrt(math.pow((x_goal - hb_x), 2) + math.pow((y_goal - hb_y), 2))
                tolerance_dist = 0.05
                tolerance_theta = 0.04
                
                e_x = x_goal - hb_x
                e_y = y_goal - hb_y
                e_theta = theta_goal - hb_theta

                # transform
                # calc. vel
                # if reached goal: stop

                if abs(distance_error) > tolerance_dist or abs(e_theta) > tolerance_theta:
                    # Translate error to robot's body frame
                    e_theta_rframe = e_theta
                    e_x_rframe = (math.cos(hb_theta)) * (e_x) + (math.sin(hb_theta)) * (e_y)
                    e_y_rframe = -(math.sin(hb_theta)) * (e_x) + (math.cos(hb_theta)) * (e_y)

                    # Calculate control velocities using P controller
                    # vel_x = pid(e_x_rframe, line_PID_params)
                    # vel_y
                    # ang_vel_z = pid(e_x_rframe, ang_PID_params)
                    vel_x = e_x_rframe * ebot_controller.kp_x
                    vel_y = e_y_rframe * ebot_controller.kp_y
                    vel_w = e_theta * ebot_controller.kp_w

                    # Limit control velocities
                    ebot_controller.vel.linear.x = float(ebot_controller.limit_vel_lin(vel_x))
                    ebot_controller.vel.linear.y = float(ebot_controller.limit_vel_lin(vel_y))
                    ebot_controller.vel.angular.z = float(ebot_controller.limit_vel_ang(vel_w))
                    ebot_controller.pub1.publish(ebot_controller.vel)
                else:
                    # Stop the robot if goal is reached
                    ebot_controller.vel.linear.x = 0.0
                    ebot_controller.vel.linear.y = 0.0  
                    ebot_controller.vel.angular.z = 0.0
                    print("Goal reached")
                    ebot_controller.pub1.publish(ebot_controller.vel)                    
                    print(f"New index = {ebot_controller.index}")
                    time.sleep(0.4)
                    
                    # Increment the goal index if goal reached
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1:
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)

        # Process ROS callbacks
        rclpy.spin_once(ebot_controller)
   
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass


import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose2D, Wrench
from std_msgs.msg import Int32
import numpy as np

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_2')

        # self.lw_pub = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        # self.rw_pub = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        # self.fw_pub = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)
        self.vel_pub=self.create_publisher(Twist,'/cmd_vel/bot1',10)
        self.vel_pub_2=self.create_publisher(Twist,'/cmd_vel/bot2',10)
        self.vel_pub_3=self.create_publisher(Twist,'/cmd_vel/bot3',10)
        
        # self.timer=self.create_timer(0.5,self.publishing)
                
        # self.rw_msg = Wrench()
        # self.lw_msg = Wrench()
        # self.fw_msg = Wrench()
        self.velocities=Twist()
    
    # def publishing(self):
    
        self.velocities.linear.y=-0.0#lw
        self.velocities.linear.x=0.0#fw
        self.velocities.linear.z=0.994*(0.0)#rw
        self.vel_pub.publish(self.velocities)
        self.vel_pub_2.publish(self.velocities)
        self.vel_pub_3.publish(self.velocities)
        # time.sleep(6.53)
        # self.velocities.linear.y=1500.0#lw
        # self.velocities.linear.x=1500.0#fw
        # self.velocities.linear.z=1500.0#rw
        # self.vel_pub.publish(self.velocities)
   
def main(args=None):
    rclpy.init(args=args)
    node=HBController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

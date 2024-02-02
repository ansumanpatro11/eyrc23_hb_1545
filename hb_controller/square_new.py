
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
        super().__init__('hb_controller_1')

        self.lw_pub = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.rw_pub = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.fw_pub = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)
        self.vel_pub=self.create_publisher(Twist,'/cmd_vel/bot1',10)
                
        self.rw_msg = Wrench()
        self.lw_msg = Wrench()
        self.fw_msg = Wrench()
        self.velocities=Twist()
        self.duration=0
        self.velocity=0.0
        self.count=0
    def move_up(self,vel):
        
        
        self.fw_msg.force.y =0.0#fw
        self.lw_msg.force.y =vel#lw
        self.rw_msg.force.y =-vel#rw

        self.velocities.linear.x =0.0#fw
        self.velocities.linear.y =vel#lw
        self.velocities.linear.z=-vel#rw
        self.count+=1
        time.sleep(3)
    def move_down(self,duration,vel):

        self.fw_msg.force.y =0.0#fw
        self.lw_msg.force.y =-vel#lw
        self.rw_msg.force.y =vel#rw
        
        self.velocities.linear.x =0.0
        self.velocities.linear.y =-vel
        self.velocities.linear.z=vel
        self.count+=1
        time.sleep(duration)
    def move_right(self,duration,vel):

        self.fw_msg.force.y =-2*vel#fw
        self.lw_msg.force.y =vel#lw
        self.rw_msg.force.y =vel#rw

        self.velocities.linear.x =-2*vel
        self.velocities.linear.y =vel
        self.velocities.linear.z=vel
        self.count+=1
        time.sleep(duration)
    def move_left(self,duration,vel):
        self.fw_msg.force.y =2*vel#fw
        self.lw_msg.force.y =-vel#lw
        self.rw_msg.force.y =-vel#rw

        self.velocities.linear.x =2*vel #fw
        self.velocities.linear.y =-vel #left
        self.velocities.linear.z=-vel  #right
        self.count+=1
        time.sleep(duration)
    def stop(self,duration):
        self.fw_msg.force.y =0.0#fw
        self.lw_msg.force.y =0.0#lw
        self.rw_msg.force.y =0.0#rw

        self.velocities.linear.x =0.0
        self.velocities.linear.y =0.0
        self.velocities.linear.z=0.0
        time.sleep(duration)
    # def publish_velocities(self):
     
    #     self.vel_pub.publish(self.velocities)
  

def main(args=None):
    rclpy.init(args=args)
    node=HBController()
    # node.velocity=50.0
    # node.duration=6
    node.count=0
    while rclpy.ok:
        
        if node.count==0:
            node.move_up(23.0)
        elif node.count==1:
            node.move_left(5,23.0)
        elif node.count==2:
            node.move_down(5,23.0)
        elif node.count==3:
            node.move_right(5,23.0)
        elif node.count==4:
            node.stop(5)
        # node.publish_velocities()
        node.vel_pub.publish(node.velocities)
        node.lw_pub.publish(node.lw_msg)
        node.rw_pub.publish(node.rw_msg)
        node.fw_pub.publish(node.fw_msg)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Pen(Node):
    def __init__(self):
        super().__init__('pen_up_down')
        self.sub1=self.create_subscription(Bool,'/pen1_down_intermediate',self.callback_1,1)
        self.sub2=self.create_subscription(Bool,'/pen2_down_intermediate',self.callback_2,1)
        self.sub3=self.create_subscription(Bool,'/pen3_down_intermediate',self.callback_3,1)

        self.pub=self.create_publisher(Bool,'/all_down',1)

        self.msg=Bool()
        
        
        self.pen1=False
        self.pen2=False
        self.pen3=False
        
    
    
    def callback_1(self, msg_1:Bool):
        self.pen1=msg_1.data
    
    def callback_2(self, msg_2:Bool):
        self.pen2=msg_2.data
    
    def callback_3(self, msg_3:Bool):
        self.pen3=msg_3.data
        



def main(args=None):
    rclpy.init(args=args)
    pen=Pen()
    while rclpy.ok():
        if pen.pen1 is True and pen.pen2 is True and pen.pen3 is True:
            
            pen.msg.data=True
            pen.pub.publish(pen.msg)
            
        else:
            pen.msg.data=False
            pen.pub.publish(pen.msg)

        rclpy.spin_once(pen)

    pen.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()





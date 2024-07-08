import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class Pen(Node):
    def __init__(self):
        super().__init__('pen_up_down')
        self.sub1=self.create_subscription(Bool,'/pen1_down_intermediate',self.callback_1,1)
        self.sub2=self.create_subscription(Bool,'/pen2_down_intermediate',self.callback_2,1)
        self.sub3=self.create_subscription(Bool,'/pen3_down_intermediate',self.callback_3,1)

        self.stop_1=self.create_subscription(Bool,'/stop_1',self.callback_stop_1,1)
        self.stop_2=self.create_subscription(Bool,'/stop_2',self.callback_stop_2,1)
        self.stop_3=self.create_subscription(Bool,'/stop_3',self.callback_stop_3,1)

        
        self.pub_pen=self.create_publisher(Bool,'/all_down',1)
        # self.pub_stop=self.create_publisher(Bool,'/Stop_all',1)

        self.msg=Bool()
        
        
        self.pen1=False
        self.pen2=False
        self.pen3=False
        
        self.stop_1=False
        self.stop_2=False
        self.stop_3=False
    
    
    def callback_1(self, msg_1:Bool):
        self.pen1=msg_1.data
    
    def callback_2(self, msg_2:Bool):
        self.pen2=msg_2.data
    
    def callback_3(self, msg_3:Bool):
        self.pen3=msg_3.data
        
        
        
    def callback_stop_1(self, msg_4:Bool):
        self.stop_1=msg_4.data
        
    def callback_stop_2(self, msg_5:Bool):
        self.stop_2=msg_5.data
        
    def callback_stop_3(self, msg_6:Bool):
        self.stop_3=msg_6.data
        
        
        
    def stop_flag_callback(self, request, response):
        print("Stop_Flag service invoked")
        response = Empty()
        return response



def main(args=None):
    rclpy.init(args=args)
    pen=Pen()
    while rclpy.ok():
        if pen.pen1 is True and pen.pen2 is True and pen.pen3 is True:
            
            pen.msg.data=True
            pen.pub_pen.publish(pen.msg)
            
        else:
            pen.msg.data=False
            pen.pub_pen.publish(pen.msg)


        if pen.stop_3 is True:
            stop_flag_service = pen.create_service(Empty, 'Stop_Flag', pen.stop_flag_callback)


        rclpy.spin_once(pen)

    pen.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()





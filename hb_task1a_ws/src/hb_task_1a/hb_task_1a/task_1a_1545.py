import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
from functools import partial

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")

        # Create publishers to control the turtles' movement
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Create subscribers to get the current pose of the turtles
        self.sub1 = self.create_subscription(Pose, '/turtle1/pose', self.callback_turtle1, 10)
        self.sub2 = self.create_subscription(Pose, '/turtle2/pose', self.callback_turtle2, 10)

        # Initialize distance counters and variables
        self.total_distance_turtle1 = 0.0
        self.total_distance_turtle2 = 0.0
        self.x_pre_turtle1 = 0
        self.y_pre_turtle1 = 0
        self.x_pre_turtle2 = 0
        self.y_pre_turtle2 = 0
        self.spawned_second_turtle = False

    def callback_turtle1(self, pose: Pose):
        msg = Twist()

        # Calculate the distance traveled by turtle1
        if self.x_pre_turtle1 != 0 and self.y_pre_turtle1 != 0:
            distance = math.sqrt((pose.x - self.x_pre_turtle1) ** 2 + (pose.y - self.y_pre_turtle1) ** 2)
            self.total_distance_turtle1 += abs(distance)

        self.x_pre_turtle1 = pose.x
        self.y_pre_turtle1 = pose.y

        if not self.spawned_second_turtle:
            # If turtle1 hasn't spawned turtle2 and completed its circle
            if self.total_distance_turtle1 >= 2.0 * math.pi:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                # Spawn turtle2 with the same parameters as turtle1
                self.spawn_second_turtle(pose.x, pose.y, pose.theta)
                self.spawned_second_turtle = True

                # Reset the distance counter for turtle1
                self.total_distance_turtle1 = 0.0
            else:
                msg.linear.x = 2.0
                msg.angular.z = 2.0

            # Publish the control commands for turtle1
            self.pub1.publish(msg)

    def callback_turtle2(self, pose: Pose):
        msg = Twist()

        # Calculate the distance traveled by turtle2
        if self.x_pre_turtle2 != 0 and self.y_pre_turtle2 != 0:
            distance = math.sqrt((pose.x - self.x_pre_turtle2) ** 2 + (pose.y - self.y_pre_turtle2) ** 2)
            self.total_distance_turtle2 += abs(distance)

        self.x_pre_turtle2 = pose.x
        self.y_pre_turtle2 = pose.y

        if self.spawned_second_turtle:
            # If turtle2 has been spawned by turtle1
            if self.total_distance_turtle2 >= 4.0 * math.pi:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 2.0
                msg.angular.z = -1.0

            # Publish the control commands for turtle2
            self.pub2.publish(msg)

    def spawn_second_turtle(self, x, y, theta):
        # Create a client to call the turtle spawn service
        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service ...")

        # Create a request to spawn turtle2 with the same parameters as turtle1
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = "turtle2"  # Set the turtle's name

        # Call the service to spawn turtle2
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()

    # Spin the node to process messages and services
    rclpy.spin(node)

    # Shutdown the node when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()

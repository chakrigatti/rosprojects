#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
    
class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller Started")
        self.alive_array=None
        self.pose=None
        self.alive_subscriber=self.create_subscription(TurtleArray,"alive_turtles",self.callback_alive_turtles,10)
        self.pose_subscriber = self.create_subscription(Pose,"turtle1/pose",self.callback_get_pose,10)
        self.publisher_=self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.timer_=self.create_timer(1,self.callback_reachdestination)
    
    def callback_reachdestination(self):
        if self.alive_array == None or len(self.alive_array) == 0:
            return
        first=self.alive_array[0]
        current = self.pose
        self.get_logger().info('The first coordinates are : '+str(first.x) + ' ' + str(first.y))
        self.get_logger().info('The current coordinates are : '+str(current.x) + ' ' + str(current.y))
        diff_x=first.x - current.x
        diff_y=first.y - current.y
        if abs(diff_x) < 0.01 and abs(diff_y) < 0.01:
            self.catch_turtle(name=first.name)
            return
        vel = Twist()
        vel.linear.x = diff_x
        vel.linear.y = diff_y
        vel.linear.z = 0.0

        vel.angular.x =0.0
        vel.angular.y =0.0
        vel.angular.z =0.0
        self.publisher_.publish(vel)
    
    def catch_turtle(self,name):
        client = self.create_client(CatchTurtle,"catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for the turtlesim node to be started.")
        request = CatchTurtle.Request()
        request.name = name
        future = client.call_async(request=request)
    
    def callback_alive_turtles(self,msg):
        self.alive_array = msg.alive_turtles

    def callback_get_pose(self,msg):
        self.pose = msg

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
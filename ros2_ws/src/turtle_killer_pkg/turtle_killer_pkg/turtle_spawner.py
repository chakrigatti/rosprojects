#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import random
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spanner")
        self.turtle_alive_list=TurtleArray()
        self.get_logger().info("Turtle Spanner started")
        self.timer_ = self.create_timer(2.0,self.span_turtle_client)
        self.service_request_lock = False
        self.alive_turtles_publisher = self.create_publisher(TurtleArray,"alive_turtles",10)
        self.alive_timer=self.create_timer(1.0,self.callback_alive_turtles_publisher)
        self.create_service(CatchTurtle,"catch_turtle",self.callback_catch_turtle)

    def callback_catch_turtle(self,request,response):
        turtle_name = request.name
        try:
            for i in range(len(self.turtle_alive_list.alive_turtles)):
                if self.turtle_alive_list.alive_turtles[i].name == turtle_name:
                    self.kill_turtle(turtle_name)
                    del self.turtle_alive_list.alive_turtles[i]
                    response.success = True
                    return response
            response.success=False
        except Exception as e:
            response.success =False
            self.get_logger().info(str(e))
        return response
    
    def kill_turtle(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for the turtlesim node to be started.")
        request = Kill.Request()
        request.name = name
        future = client.call_async(request=request)
        future.add_done_callback(self.callback_kill_turtle)
    
    def callback_kill_turtle(self,future):
        pass


    def callback_alive_turtles_publisher(self):
        msg = self.turtle_alive_list
        self.alive_turtles_publisher.publish(msg)

    def span_turtle_client(self):
        if self.service_request_lock or len(self.turtle_alive_list.alive_turtles)>4 :
            return
        
        client = self.create_client(Spawn , "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for the turtlesim node to be started.")
            self.service_request_lock = True
        
        self.service_request_lock = False
        
        request = Spawn.Request()
        x = float(random.randint(0,11))
        y = float(random.randint(0,11))
        theta = round(random.uniform(0, 6.28),2)
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request=request)
        future.add_done_callback(
            partial(self.callback_spawn , x=x,y=y,theta=theta)
        )

    def callback_spawn(self, future, x,y,theta):
        try:
            response = future.result()
            turtle = Turtle()
            turtle.x = x
            turtle.y = y
            turtle.theta =theta
            turtle.name = response.name
            self.turtle_alive_list.alive_turtles.append(turtle)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
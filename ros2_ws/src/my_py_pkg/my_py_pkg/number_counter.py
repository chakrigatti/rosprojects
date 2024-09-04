#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
    
    
class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.subscriber_ = self.create_subscription(Int64 , "number" ,self.callback_count_number,10)
        self.publisher_ = self.create_publisher(Int64 , "number_count", 10)
        self.get_logger().info("Number counter has been started.")
    
    def callback_count_number(self,msg):
        self.counter_ += msg.data
        counter = Int64()
        counter.data = self.counter_
        self.get_logger().info("Publishing the counter with value : "+ str(self.counter_))
        self.publisher_.publish(counter)

    
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
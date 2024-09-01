import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import tf_transformations

class PoseCirclePublisher(Node):
    def __init__(self):
        super().__init__('pose_circle_publisher')
        self.publisher_=self.create_publisher(PoseStamped,'pose_circle',10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period , self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        radius = 2.0
        speed = 0.1
        center_x = 0.0
        center_y = 0.0

        msg.pose.position.x = center_x + radius * math.cos(self.i * speed)
        msg.pose.position.y = center_y + radius * math.sin(self.i * speed)
        msg.pose.position.z = 0.0

        orientation = tf_transformations.quaternion_from_euler(0,0,self.i * speed)
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]

        self.publisher_.publish(msg)
        self.i +=1

def main(args=None):
    rclpy.init(args=args)
    pose_circle_publisher = PoseCirclePublisher()
    rclpy.spin(pose_circle_publisher)
    pose_circle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

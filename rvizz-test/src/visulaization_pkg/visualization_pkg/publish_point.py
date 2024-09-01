import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time

class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')
        self.publisher_=self.create_publisher(PointStamped,'point_topic',10)
        timer_period=1.0
        self.timer = self.create_timer(timer_period,self.publish_point)

    def publish_point(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id= "map"
        msg.point.x = 1.0
        msg.point.y = 2.0
        msg.point.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.point)


def main(args=None):
    rclpy.init(args=args)
    node=PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
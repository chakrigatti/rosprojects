import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher to /joint_states topic
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to call the callback function periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initializing joint states
        self.joint_state = JointState()
        self.joint_state.name = ['joint_1', 'joint_2']
        self.joint_state.position = [0.0, 0.0]
        self.angle = 0.0

    def timer_callback(self):
        # Update the joint position
        self.angle += 0.05
        self.joint_state.header = Header()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Assigning sine wave motion to joints for visualization
        self.joint_state.position[0] = math.sin(self.angle)
        self.joint_state.position[1] = math.cos(self.angle)

        # Publish the joint states
        self.publisher.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    
    rclpy.spin(joint_controller)

    # Shutdown when done
    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

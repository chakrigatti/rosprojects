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
        self.joint_state.name = [
            'link1_to_link2', 
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_link6',
            'link6_to_link6flange',
            'gripper_controller',
            'gripper_base_to_gripper_left2',
            'gripper_left3_to_gripper_left1',
            'gripper_base_to_gripper_right3',
            'gripper_base_to_gripper_right2',
            'gripper_right3_to_gripper_right1'
        ]
        self.joint_count = len(self.joint_state.name)
        self.joint_state.position = [0.0] * self.joint_count
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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle,GoalStatus
from my_robot_interfaces.action import MoveRobot
from example_interfaces.msg import Empty


class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client")
        self.count_until_client_ = ActionClient(self,MoveRobot,"move_robot")
        self.subscriber_ = self.create_subscription(Empty,"cancel_move",self.cancel_goal,10)
        
    def send_goal(self,position,velocity):
        self.count_until_client_.wait_for_server()

        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity

        self.get_logger().info("Sending the goal")
        self.count_until_client_. \
            send_goal_async(goal,feedback_callback=self.goal_feedback_callback). \
            add_done_callback(self.goal_response_callback)
        
    def cancel_goal(self,msg):
        pass

    def goal_response_callback(self,future):
        self.goal_handle_:ClientGoalHandle  = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self,future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Cancelled")
        
        self.get_logger().info(result.message)
        self.get_logger().info("Reached : "+str(result.position))

    def goal_feedback_callback(self,feedback_msg):
        current_position = feedback_msg.feedback.current_position
        self.get_logger().info("Got Feedback " + str(current_position))

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode()
    node.send_goal(10,1)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
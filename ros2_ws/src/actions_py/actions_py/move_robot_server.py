#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from my_robot_interfaces.action import MoveRobot

import time
import threading

    
class MoveRobotNode(Node):
    def __init__(self):
        super().__init__("move_robot_server")
        self.current_postion_ = 50
        self.goal_handle_:ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Started the move robot action server")
    
    def goal_callback(self,goal_request:MoveRobot.Goal):
        self.get_logger().info("Recieved the goal")

        # Validation
        if (goal_request.position < 0 or goal_request.position >100 ) \
            and (goal_request.velocity <0 or goal_request.velocity >100):
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Policy : prempt a active goal if a new goal comes
        with self.goal_lock_ :
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort the current goal and accepting the new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Recieved the cancel request")
        return CancelResponse.ACCEPT


    def execute_callback(self,goal_handle:ServerGoalHandle):
        
        with self.goal_lock_:
            self.goal_handle_=goal_handle
        
        # Getting values
        position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()
        while position !=self.current_postion_:
            if not goal_handle.is_active:
                result.position = self.current_postion_
                result.message = "The goal has been aborted."
                return result
            if goal_handle.is_cancel_requested :
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.position = self.current_postion_
                result.message = "The goal has been cancelled."
                return result
            self.get_logger().info("Moving from the postion : "+str(self.current_postion_))
            time.sleep(1.0)
            diff = position - self.current_postion_
            if abs(diff)<=velocity:
                self.current_postion_ += diff 
            else:
                self.current_postion_ += int((diff/abs(diff)))*velocity
            self.get_logger().info("Moving to the postion : "+str(self.current_postion_))
            feedback.current_position = self.current_postion_
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.position = self.current_postion_
        result.message = "Moved to the target position."
        return result


    
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    rclpy.spin(node , MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
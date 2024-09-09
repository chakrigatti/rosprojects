#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil
from rclpy.action import ActionServer,GoalResponse , CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup 

import time
    
class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_:ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_=[]
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
            )
        self.get_logger().info("Action server started")

    def goal_callback(self,goal_request:CountUntil.Goal):
        self.get_logger().info("Recieved the goal")

        # Policy : refuse goal if current goal is active
        # with self.goal_lock_:
        #     if self.goal_handler_ is not None and self.goal_handler_.is_active:
        #         self.get_logger().info("A Goal is already active, so recjecting the goal.")
        #         return GoalResponse.REJECT


        # Validate the goal request

        if goal_request.target_number <=0 :
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # # Policy : prempt a active goal if a new goal comes
        # with self.goal_lock_ :
        #     if self.goal_handler_ is not None and self.goal_handler_.is_active:
        #         self.get_logger().info("Abort the current goal and accept the new goal")
        #         self.goal_handler_.abort()

        
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self,goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Recieved the cancel request")
        return CancelResponse.ACCEPT


    def execute_callback(self,goal_handle:ServerGoalHandle):
        # Get request from the goal
        with self.goal_lock_:
            self.goal_handle_=goal_handle
        
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        counter =0
        result = CountUntil.Result()
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal()
                return result
            counter+=1
            self.get_logger().info(str(counter))
            feedback.current_number=counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        
        # Once done set goal final state
        goal_handle.succeed()
        result.reached_number = counter
        self.process_next_goal()
        return result
    
    def process_next_goal(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node , MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
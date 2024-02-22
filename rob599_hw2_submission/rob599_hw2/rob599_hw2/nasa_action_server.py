#!/usr/bin/env python3

# This node emulates a NASA rocket launch using an action server. 
    # It takes a goal (# of sec until launch), issues feedback (counting down to zero),
    # and completes the goal (reporting "launched!")
    # It also allows the launch to be aborted before the countdown hits zero

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rob599_hw2_msgs.action import Nasa
import time

class NasaActionServer(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__('nasa')
        self.get_logger().info("Started NASA node")

        self.action_server = ActionServer(self, Nasa, 'launch_rocket', 
                                          execute_callback=self.action_callback,
                                          cancel_callback=self.cancel_callback)
        self.get_logger().info("Started NASA action server")

    def cancel_callback(self, goal_handle):
        """Callback that triggers on cancel request. Cancels the action"""
        self.get_logger().info("Recieved cancel request")
        return CancelResponse.ACCEPT
    
    def action_callback(self, goal_handle):
        self.get_logger().info(f"Goal recieved: {goal_handle.request.sec_to_launch}sec")

        # set up the feedback message
        feedback_msg = Nasa.Feedback()

        # start executing the goal
        for i in range(0, goal_handle.request.sec_to_launch):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal Canceled")
                return Nasa.Result()
            
            # each iteration, decrease the countdown and publish the feedback
            feedback_msg.countdown = goal_handle.request.sec_to_launch - i
            self.get_logger().info(f"Publishing feedback: {feedback_msg.countdown}")
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        result = Nasa.Result()
        result.outcome = "Launched!"
        
        # Let the action server know that we've succeeded in the action.
        goal_handle.succeed()
        self.get_logger().info(f"Result: {result.outcome}")
        
        return result
    
def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate
    nasa = NasaActionServer()
    executor = MultiThreadedExecutor()
    # give control over to ros
    rclpy.spin(nasa, executor=executor)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()


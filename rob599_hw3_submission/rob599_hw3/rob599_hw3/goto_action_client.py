#!/usr/bin/env python3

# Action client that demonstrates the behavior of the go_to action in the places node

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rob599_hw3_msgs.action import GoTo

class GoToClient(Node):
    def __init__(self):
        # initialize the node
        super().__init__("goto_client")
        # initialize the action client
        self.action_client = ActionClient(self, GoTo, "go_to")
        self.get_logger().info("Waiting for action server")
        self.action_client.wait_for_server()

    def send_goal(self, set_goal:str):
        """Method to format and send the given goal to the action server"""
        # format the goal
        goal_msg = GoTo.Goal()
        goal_msg.position_name = set_goal

        # send the goal
        self.get_logger().info(f"Sending goal request: {goal_msg.position_name}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback):
        """Callback that fires when we get feedback"""
        self.get_logger().info(f"{feedback.feedback.distance_to_goal}m to goal")

    def goal_response_callback(self, future):
        """Callback that fires when the action is accepted or rejected"""
        # get the result of requesting the action and hold onto it
        request_handle = future.result()
        self._request_handle = request_handle

        # log if the action was rejected
        if not request_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        # if accepted, grab a handle to the result and give it a callback for processing
        self.result_handle = request_handle.get_result_async()
        self.result_handle.add_done_callback(self.process_result_callback)

    def process_result_callback(self, future):
        """Callback that fires when we get a result"""
        # Grab the result and log it
        result = future.result().result
        self.get_logger().info(f"Navigation outcome: {result.outcome}")
        # shutdown after recieving the result
        rclpy.shutdown()

def main(args=None):
    # initialize
    rclpy.init(args=args)
    # grab the goal position from the command line, if one is given
    try:
        goal_position = str(sys.argv[1])
    except:
        goal_position = "start"
    # instantiate
    my_client = GoToClient()
    # send goal
    my_client.send_goal(goal_position)
    # hand control over to ros
    rclpy.spin(my_client)
    # shutdown cleanly, in case we didn't do that after recieving the result
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
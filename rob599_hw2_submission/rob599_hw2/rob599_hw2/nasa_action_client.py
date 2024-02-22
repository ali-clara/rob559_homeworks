#!/usr/bin/env python3

# Action client that demonstrates the behavior of the Nasa action server

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rob599_hw2_msgs.action import Nasa

class NasaClient(Node):
    def __init__(self, launch_abort=None):
        # initialize the node
        super().__init__('nasa_client')
        # initialize the action client (same name and type as the server)
        self.action_client = ActionClient(self, Nasa, 'launch_rocket')
        self.launch_abort = launch_abort

    def send_goal(self, set_goal):
        # wait for the action server
        self.get_logger().info("Waiting for action server")
        self.action_client.wait_for_server()
        
        # format the goal
        goal_msg = Nasa.Goal()
        goal_msg.sec_to_launch = set_goal

        # send the goal
        self.get_logger().info(f"Sending goal request: {goal_msg.sec_to_launch}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        """Callback that fires when we get feedback"""
        self.get_logger().info(f"{feedback.feedback.countdown} seconds to launch")

    def goal_response_callback(self, future):
        """Callback that fires when the action is accepted or rejected"""
        # get the result of requesting the action, and hold onto it
        request_handle = future.result()
        self._request_handle = request_handle

        # log if the action was rejected
        if not request_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        # if accpeted, grab a handle to the result and give it a callback for processing
        self.result_handle = request_handle.get_result_async()
        self.result_handle.add_done_callback(self.process_result_callback)

        # create a timer to cancel the goal after a given number of seconds
        if self.launch_abort is not None:
            self._cancel_timer = self.create_timer(self.launch_abort, self.cancel_timer_callback)

    def process_result_callback(self, future):
        """Callback that fires when we get a result"""
        # Grab the result and log it
            # (has a type that corresponds to the result field of the action definiton)
        result = future.result().result
        self.get_logger().info(f"Launch outcome: {result.outcome}")
        # shutdown after recieving the result
        rclpy.shutdown()

    def cancel_timer_callback(self):
        """A quick timer that fires once after the goal is accepted. Triggers
            a callback that cancels the goal after self.launch_abort seconds"""
        future = self._request_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        self._cancel_timer.cancel()

    def cancel_done(self, future):
        """Cancels the goal and shuts down"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successuflly canceled")
        else:
            self.get_logger().info("Goal failed to cancel")
        
        rclpy.shutdown()
    
def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate
    try:
        launch_abort = int(sys.argv[1])
    except:
        launch_abort = None
    action_client = NasaClient(launch_abort)
    # send goal
    action_client.send_goal(10)
    # hand control over to ros
    rclpy.spin(action_client)
    # shutdown cleanly, in case we didn't do that after recieving the result
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        

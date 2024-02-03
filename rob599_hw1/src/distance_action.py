#!/usr/bin/env python3

# A node to provide feedback as the Fetch is driving to its goal

import rospy
import actionlib
import sys
from rob599_hw1.msg import SetDistanceAction, SetDistanceGoal, SetDistanceResult

class ActionCall():
    def __init__(self) -> None:
        # Initialize the node
        rospy.init_node('action_client', argv=sys.argv)

        # Create an action client and wait until it has connected successfully to the server
        self.client = actionlib.SimpleActionClient('driving_action', SetDistanceAction)
        self.client.wait_for_server()
        rospy.loginfo("Set action client")

    def done_callback(self, status, result):
        """When the action is done, checks success/fail status and reports along with the
            current distance to the wall"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f'Suceeded, robot is {result.distance_from_wall}m from the wall')
        else:
            rospy.loginfo(f'Failed, robot is {result.distance_from_wall}m from the wall')

    def feedback_callback(self, feedback):
        """Upon feedback, logs the distance left to go"""
        rospy.loginfo(f"{feedback.distance_to_go}m left to go")

    def active_callback(self):
        rospy.loginfo('Action is active')

    def run(self):
        """Gets the goal from the command line and sends it to the action"""
        # Get the command line argument, if one exists. Default to 1m
        try:
            dist = float(sys.argv[1])
            print(dist)
        except:
            dist = 1.0
        # send the goal and wait for the result
        rospy.loginfo(f"Setting goal to {dist}m")
        goal = SetDistanceGoal(stopping_distance=dist)
        self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()

if __name__ == "__main__":
    my_action = ActionCall()
    my_action.run()
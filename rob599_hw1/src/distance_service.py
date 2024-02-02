#!/usr/bin/env python3

# A node to let us set the distance in front of an obstacle for the Fetch to stop at

# import the things
import rospy
import sys
from rob599_hw1.srv import SetDistance

class ServiceCall():
    def __init__(self):
        # initialize the node
        rospy.init_node("service_client", argv=sys.argv)

        # Hang out until the service has been set up
        rospy.wait_for_service("stopping_distance")

        # set up service proxy with name (stopping_distance) and base service message type (SetDistance) 
            # to access the service. Service was defined in drive_forward.py
        self.stopping_service = rospy.ServiceProxy("stopping_distance", SetDistance)

    def test_service(self):
        """Tests that the service defined in drive_forward.py sets and runs with no exceptions"""
        for i in range(5):
            try:
                answer = self.stopping_service(i)
                rospy.loginfo(f"Sent {i} and got {answer.confirmation}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed for {i}: {e}")

    def run(self):
        """Runs the node. Asks for a user input from the command line and passes it to
            the 'stopping_distance' service. Checks to make sure the input is a real, positive number"""
        while not rospy.is_shutdown():
            # set up some flags for valid answers
            real_number = False
            positive = False

            # ask the command line for a stopping distance
            dist_to_wall = input("Input a distance to the wall (m): ")

            # check if the input can be cast to a float
            try:
                dist_to_wall = float(dist_to_wall)
                real_number = True
                # check if the number is positive
                if dist_to_wall < 0:
                    print("Please input a positive number.")
                    positive = False
                else:
                    positive = True
            except:
                print("Please enter a real number.")
                real_number = False
            
            if real_number and positive:
                try:
                    answer = self.stopping_service(dist_to_wall)
                    rospy.loginfo(f"Sent {dist_to_wall}, got {answer.confirmation}")
                except rospy.ServiceException as e:
                    rospy.logwarn(f"Service call failed for {dist_to_wall}: {e}")

if __name__ == "__main__":
    my_service_call = ServiceCall()
    my_service_call.run()

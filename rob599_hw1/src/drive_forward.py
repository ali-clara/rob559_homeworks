#!/usr/bin/env python3

# Drives the Fetch forward until it's a set distance away from the nearest obstacle
# subscribes to laser rangefinder messages (type LaserScan)
# publishes messages (type Twist) to cmd_vel

# import the ROS stuff
import rospy
import sys
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rob599_hw1.srv import SetDistance, SetDistanceResponse
from rob599_hw1.msg import SetDistanceAction, SetDistanceActionFeedback, SetDistanceActionResult

# import the other stuff
import numpy as np

class DriveForward():
    def __init__(self) -> None:
        # Initialize the node
        rospy.init_node('drive_forward', argv=sys.argv)
        
        # Set up the publisher and subscriber. 
        # The Fetch will listen for Twist messages on the cmd_vel topic.
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('trimmed_scan', LaserScan, self.laser_callback)
        rospy.loginfo("Started drive-to-wall publisher and subscriber")
        
        # Set up the service. This allows us to set the distance in front of an obstacle
            # the Fetch will stop at
        self.distance_service = rospy.Service('stopping_distance', SetDistance, self.dist_service_callback)
        rospy.loginfo("Started wall distance service")

        # Set up the action. This will provide feedback as the Fetch approaches its goal
        self.drive_action = actionlib.SimpleActionServer('driving_action', SetDistanceAction, self.dist_action_callback, False)
        self.drive_action.start()
        rospy.loginfo("Started wall distance action")

        # Initialize class variables
        self.stopping_distance = 1.0
        self.laser_ranges = None
        self.laser_angles = None
        self.range_min = None

    def laser_callback(self, msg):
        """Callback function for the laser subscriber. Called every time a new
            message (type LaserScan) is recieved to 'base_scan'.
            Saves the angle range and the laser range to class variables as np arrays"""
        self.laser_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        self.laser_ranges = np.array(msg.ranges)
        self.range_min = min(self.laser_ranges)
        rospy.loginfo(f"Closest laser scan: {self.range_min}m")

    def dist_service_callback(self, request):
        """Callback function for the stopping distance service. Sets the class variable
            based on the input given (request.SetDistance). Returns True if successfully set."""
        rospy.loginfo(f"Stopping distance service got {request.distance}")
        self.stopping_distance = request.distance
        return SetDistanceResponse(True)
    
    def dist_action_callback(self, goal):
        """Callback function for the action distance service"""

        # set the distance to go based on the action
        self.stopping_distance = goal

        # find how much further we have to travel and publish it as feedback
        dist_to_go = self.range_min - self.stopping_distance
        self.drive_action.publish_feedback(SetDistanceActionFeedback(distance_to_go=dist_to_go))

        # if we recieved a new goal, preempt the old goal
        if self.drive_action.is_new_goal_available():
            self.drive_action.set_preempted(SetDistanceActionResult(distance_to_wall=self.range_min))
            return
        
        # if we've reached the wall (or are close enough), return succeeded
        if self.found_goal():
            self.drive_action.set_succeeded(SetDistanceActionResult(distance_to_wall=self.range_min))

    # def find_goal(self):
    #     """Finds the x distance and angle goal in the Fetch's coordinate frame"""
    #     min_laser_index = np.argmin(self.laser_ranges)
    #     self.angle_goal = self.laser_angles[min_laser_index]
    #     self.range_min = self.laser_ranges[min_laser_index]
    #     rospy.loginfo(f"Closest laser scan: {np.round(self.range_min,3)}m at {np.round(np.rad2deg(self.angle_goal),3)}deg")
    
    def found_goal(self):
        """Returns True if the minimum range measurement is within an 
            accpetable threshold of the set stopping distance"""
        # are we within an acceptable threshold of the goal?
        if abs(self.range_min - self.stopping_distance) < 0.1:
            return True
        else:
            return False
    
    def calculate_velocity(self): 
        """Maps the x distance (range) goal and the angle goal to velocities"""
        # set up an linear relationship to evaluate the x velocity at each distance value 
            # and cap at 1m/s
        x_velocity = 0.25*(self.range_min - self.stopping_distance)
        x_velocity = min(x_velocity, 1)

        return x_velocity
        
    def create_fetch_twist(self, linear_x):
        """Formats a Twist message for the Fetch, filling in the linear x velocity
            and the angular z velocity"""
        cmd = Twist()

        # Linear velocities: m/s
        cmd.linear.x = min(linear_x, 1) # make sure we cap at 1m/s
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        # Rotational velocities: rad/sec 
        # The Fetch will only respond to rotations around the z (vertical) axis
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        # cmd.angular.z = min(angular_z, 0.25*(2*np.pi))   # make sure we cap at 0.25 rev/s

        return cmd
    
    def run(self):
        """Run the node"""
        rate = rospy.Rate(10)
        # hang out until we've recieved a message
        rospy.wait_for_message('trimmed_scan', LaserScan, timeout=10)
        while not rospy.is_shutdown():
            # self.find_goal()
            x_velocity = self.calculate_velocity()
            cmd = self.create_fetch_twist(x_velocity)
            rospy.loginfo(f"Publishing {np.round(cmd.linear.x,3)}m/s")
            self.publisher.publish(cmd)

            rate.sleep()

if __name__ == "__main__":
    my_drive = DriveForward()
    my_drive.run()
#!/usr/bin/env python3

# Drives the Fetch forward until it's 1m away from the nearest obstacle
# subscribes to laser rangefinder messages (type LaserScan)
# publishes messages (type Twist) to cmd_vel

# import the ROS stuff
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rob599_hw1.srv import SetDistance, SetDistanceResponse

# import the other stuff
import numpy as np

class DriveForward():
    def __init__(self) -> None:
        # Initialize the node
        rospy.init_node('mover', argv=sys.argv)
        
        # Set up the publisher and subscriber. 
        # The Fetch will listen for Twist messages on the cmd_vel topic.
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.laser_callback)
        rospy.loginfo("Started drive-to-wall publisher and subscriber")
        
        # Set up the service. This allows us to set the distance in front of an obstacle
            # the Fetch will stop at
        self.distance_service = rospy.Service('stopping_distance', SetDistance, self.dist_service_callback)
        rospy.loginfo("Started wall distance service")

        # Initialize class variables
        self.stopping_distance = 1.0

        self.laser_ranges = None
        self.laser_angles = None
        self.range_goal = None
        self.angle_goal = None

    def laser_callback(self, msg):
        """Callback function for the laser subscriber. Called every time a new
            message (type LaserScan) is recieved to 'base_scan'.
            Saves the angle range and the laser range to class variables as np arrays"""
        self.laser_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        self.laser_ranges = np.array(msg.ranges)

    def dist_service_callback(self, request):
        """Callback function for the stopping distance service. Sets the class variable
            based on the input given (request.SetDistance). Returns True if successfully set."""
        rospy.loginfo(f"Stopping distance service got {request.distance}")
        self.stopping_distance = request.distance
        return SetDistanceResponse(True)

    def find_goal(self):
        """Finds the x distance and angle goal in the Fetch's coordinate frame"""
        min_laser_index = np.argmin(self.laser_ranges)
        self.angle_goal = self.laser_angles[min_laser_index]
        self.range_goal = self.laser_ranges[min_laser_index]
        rospy.loginfo(f"Closest laser scan: {np.round(self.range_goal,3)}m at {np.round(np.rad2deg(self.angle_goal),3)}deg")
    
    def calculate_velocity(self): 
        """Maps the x distance (range) goal and the angle goal to velocities"""
        # set up an interpolation to evaluate the x velocity at each distance value 
            # and cap at 1m/s. Equivalent to y = 0.25*(x-1) between 0<x<5
        x_distance_between = [0, 5]
        x_velocity_between = [-0.25, 1]
        x_velocity = np.interp(self.range_goal, x_distance_between, x_velocity_between)

        # set up an interpolation to evaluate the z velocity, caps at 0.15rev/s
        z_angle_between = [-1.92, 1.92]
        z_velocity_bewteen = [-0.15*(2*np.pi), 0.15*(2*np.pi)]
        z_velocity = np.interp(self.angle_goal, z_angle_between, z_velocity_bewteen)

        return x_velocity, z_velocity
        
    def create_fetch_twist(self, linear_x, angular_z):
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
        cmd.angular.z = min(angular_z, 0.25*(2*np.pi))   # make sure we cap at 0.25 rev/s

        return cmd
    
    def run(self):
        """Run the node"""
        rate = rospy.Rate(10)
        # hang out until we've recieved a message
        rospy.wait_for_message('base_scan', LaserScan, timeout=10)
        while not rospy.is_shutdown():
            self.find_goal()
            x_velocity, z_velocity = self.calculate_velocity()
            cmd = self.create_fetch_twist(x_velocity, z_velocity)
            rospy.loginfo(f"Publishing {np.round(cmd.linear.x,3)}m/s, {np.round(cmd.angular.z,3)}rad/s")
            self.publisher.publish(cmd)

            rate.sleep()

if __name__ == "__main__":
    my_drive = DriveForward()
    my_drive.run()
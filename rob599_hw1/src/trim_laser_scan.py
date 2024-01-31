#!/usr/bin/env python3

# Trims laser scan messages to only those in front of the Fetch
# subscribes to laser rangefinder messages (type LaserScan)
# publishes laser rangerinder messages (type LaserScan) to \base_scan

# import the ROS stuff
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# import the other stuff
import numpy as np

class TrimLaser():
    def __init__(self):
        # Initialize the node
        rospy.init_node('trim_msg', argv=sys.argv)

        # Set up publisher and subscriber
        self.publisher = rospy.Publisher('trimmed_scan', LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.laser_callback)

        self.laser_angles = None
        self.laser_ranges = None

        self.my_laser_msg = LaserScan()
        
    def laser_callback(self, msg):
        """Callback function for the laser subscriber. Called every time a new
            message (type LaserScan) is recieved. \\
            Saves the laser scan angles and ranges to class variables as np arrays"""
        self.laser_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        self.laser_ranges = np.array(msg.ranges)

        self.my_laser_msg = msg

    def trim_message(self):
        """Clips the laser scan messages to only be in front of the Fetch,
            assuming that the Fetch is 1m wide"""
        # finds the x coordinate corresponding to each range
        hz_distance = self.laser_ranges*np.sin(self.laser_angles)
        # if the x coordinate is further than 0.5 (half the width of the Fetch)
            # from the origin (center of the Fetch), discard it
        distance_threshold = 0.5
        mask = np.abs(hz_distance) > distance_threshold
        self.laser_ranges = self.laser_ranges[mask]
        self.laser_angles = self.laser_angles[mask]

        self.my_laser_msg.ranges = self.laser_ranges
    
    def run(self):
        """Runs the node"""
        rate = rospy.Rate(10)
        # hang out until we've recieved a message
        rospy.wait_for_message('base_scan', LaserScan, timeout=10)
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing trimmed laser scan")
            self.trim_message()
            self.publisher.publish(self.my_laser_msg)
            rate.sleep()

if __name__ == "__main__":
    my_laser = TrimLaser()
    my_laser.run()
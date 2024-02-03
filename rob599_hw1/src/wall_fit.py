#!/usr/bin/env python3

# Node that fits a line to the wall in front of the Fetch

# import the things
import numpy as np
import rospy
import sys
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LineFit():
    def __init__(self) -> None:
        # initialize the node
        rospy.init_node('wall_fit', argv=sys.argv)

        self.laser_subscriber = rospy.Subscriber('trimmed_scan', LaserScan, self.laser_callback)
        self.marker_publisher = rospy.Publisher('wall_fit_marker', Marker, queue_size=10)
        rospy.loginfo("Intialized wall-fit publisher and subscriber")

        self.range_min = None
        self.angle_min = None

        self.range_max = None
        self.angle_max = None

    def laser_callback(self, msg):
        """Callback function for the laser subscriber. Called every time a new
            message (type LaserScan) is recieved to 'base_scan'.
            Saves the angle range and the laser range to class variables as np arrays"""
        laser_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        laser_ranges = np.array(msg.ranges)

        mask = laser_ranges < np.inf
        laser_angles = laser_angles[mask]
        laser_ranges = laser_ranges[mask]

        min_index = np.argmin(laser_ranges)
        self.range_min = laser_ranges[min_index]
        self.angle_min = laser_angles[min_index]

        max_index = np.argmax(laser_ranges)
        self.range_max = laser_ranges[max_index]
        self.angle_max = laser_angles[max_index]

    def polar_to_cartesian(self, r, angle):

        rospy.loginfo(f"point at {r}m, {angle}rad")
        x = r*np.cos(angle)
        y = r*np.sin(angle)

        return x,y
    
    def create_point(self, x, y):
        point = Point()
        
        point.x = x
        point.y = y
        point.z = 0.0

        rospy.loginfo(f"creating point at {x}, {y}")

        return point
    
    def create_marker(self, points):
        """Creates a marker of a line strip between the points given. 
            points - list of geometry_msg.Point objects"""
        marker = Marker()
        marker.header.frame_id = 'laser_link'
        marker.header.stamp = rospy.Time()

        marker.type = 4 # line strip
        marker.action = 0   # add marker

        marker.points = points
        
        # marker.pose.position.x = 0
        # marker.pose.position.y = 0
        # marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        # marker.scale.y = 1.0
        # marker.scale.z = 1.0
        
        marker.color.a = 1.0    # making it visible!
        marker.color.g = 1.0

        return marker
    
    def run(self):
        """Runs the node"""
        # hang out until we've recieved a message
        rospy.wait_for_message('trimmed_scan', LaserScan, timeout=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            xmin, ymin = self.polar_to_cartesian(self.range_min, self.angle_min)
            xmax, ymax = self.polar_to_cartesian(self.range_max, self.angle_max)

            points = [self.create_point(xmin, ymin), self.create_point(xmax, ymax)]
            marker = self.create_marker(points)

            self.marker_publisher.publish(marker)
            rate.sleep()
        

if __name__ == "__main__":
    my_wall_fit = LineFit()
    my_wall_fit.run()
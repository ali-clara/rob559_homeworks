#!/usr/bin/env python3

# This node has the service interface "memorize_position" that records the current robot's
# pose and vizualizes the saved poses as markers

# ROS imports
import rclpy
from rclpy.node import Node 
from visualization_msgs.msg import Marker
from rob599_hw3_msgs.srv import MemorizePosition, ClearPositions

# standard imports
import numpy as np

class Places(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("places")
        self.get_logger().info("Started places node")

        # set up services



    def create_marker_msg(self, type:int, position, scale, frame='/map', color=[1.0,0.0,0.0,1.0], text=None) -> Marker:
        """Method to create a marker given input parameters"""
        marker = Marker()

        # set the appropriate marker fields
        marker.type = type
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.markers_published
        self.markers_published += 1
        # add the marker
        marker.action = 0

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        if text is not None:
            marker.text = text

        return marker
    
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class
    my_places = Places()
    # hand control over to ros
    rclpy.spin(my_places)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()
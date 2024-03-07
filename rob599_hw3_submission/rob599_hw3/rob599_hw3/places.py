#!/usr/bin/env python3

# This node has the service interface "memorize_position" that records the current robot's
# pose and vizualizes the saved poses as markers

# ROS imports
import rclpy
from rclpy.node import Node 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from rob599_hw3_msgs.srv import MemorizePosition, ClearPositions, Save, Load

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# standard imports
import numpy as np

class Places(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("places")
        self.get_logger().info("Started places node")

        # set up services
        self.position_server = self.create_service(MemorizePosition, 'memorize_position', self.position_service_callback)
        self.get_logger().info("Started memorize_position service")
        self.clear_positions_server = self.create_service(ClearPositions, 'clear_positions', self.clear_positions_callback)
        self.get_logger().info("Started clear_positions service")
        self.save_positions = self.create_service(Save, 'save', self.save_pos_callback)
        self.get_logger().info("Started save service")
        self.load_positions = self.create_service(Load, 'load', self.load_pos_callback)

        # set up marker publisher
        self.marker_pub = self.create_publisher(Marker, 'recorded_positions', 1)
        self.markers_published = 0
        self.positions_recorded = {}

        # set up some tf stuff
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def position_service_callback(self, request, response):
        """Callback function for the memorize_position service. Saves the current 
            turtlebot location to a dictionary and drops a marker
        """
        marker_name = request.place_name
        try:
            current_pose = self.get_current_pose("map")
            self.get_logger().info(f"Got current pose in world frame: {current_pose}")

            marker = self.create_marker_msg(marker_type=2, pose=current_pose)
            marker_txt = self.create_marker_msg(marker_type=9, pose=current_pose, text=marker_name)
            self.marker_pub.publish(marker)
            self.marker_pub.publish(marker_txt)
            self.positions_recorded.update({marker_name: current_pose})
            response.result = True
        except TransformException as e:
            self.get_logger().info(f"Transform failed: {e}")
            response.result = False

        return response

    def clear_positions_callback(self, request, response):
        """Callback function for the clear_positions service. Clears all markers
            and the dictionary the positions were saved to"""
        if request.clear:
            self.get_logger().info(f"Clearning all {self.markers_published} markers")
            self.remove_all_markers()
            self.positions_recorded = {}

        response.result = True
        return response
    
    def save_pos_callback(self, request, response):
        """Callback for the save_positions service. Saves the dictionary of saved places
            to a yaml file based on the given name"""
        filename = request.filename

    def load_pos_callback(self, request, response):
        """Callback for the load_positions service. Loads the yaml file."""
        filename = request.filename
    
    def get_current_pose(self, frame_id):
        """Method that builds a stamped pose for the turtlebot base link origin
            and returns that pose in the given frame
            Args - frame_id (str): frame you want the turtlebot position in
        """
        # build a stamped pose and set the frame id
        origin = PoseStamped()
        origin.header.frame_id = 'base_link'

        # set the position
        origin.pose.position.x = 0.0
        origin.pose.position.y = 0.0
        origin.pose.position.z = 0.0
        
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0 # arbitrary orientation

        # get the transform to the map frame. Raises an exception if it fails, which
        # is dealt with in the try/except block in the callback
        new_pose = self.tf_buffer.transform(origin, frame_id, rclpy.duration.Duration(seconds=1))

        return new_pose.pose
    
    def remove_all_markers(self, frame='/map'):
        """Method to remove all markers from the given frame"""
        for i in range(self.markers_published):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = frame
            marker.action = Marker.DELETEALL
            self.marker_pub.publish(marker)
    
    def create_marker_msg(self, marker_type:int, pose, frame='/map', scale=[0.1,0.1,0.1], color=[1.0,0.0,0.0,1.0], text=None) -> Marker:
        """Method to create a marker given input parameters"""
        marker = Marker()

        # set the appropriate marker fields
        marker.type = marker_type
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.markers_published
        self.markers_published += 1
        # add the marker
        marker.action = 0

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.pose = pose

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
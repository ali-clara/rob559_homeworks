#!/usr/bin/env python3

# This node checks the rotational and linear speed of a robot. It subscribes
# to Twist messages from the "speed_in" topic and, every 30 seconds, reports
# how many messages recieved were outside the set limits

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class SpeedChecker(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("speed_checker")
        self.get_logger().info("Started speed checker")

        # create a subscriber and a timer
        self.speed_sub = self.create_subscription(Twist, "speed_in", self.speed_callback, 10)
        self.check_timer = self.create_timer(30, self.timer_callback)

        # set node parameters
        self.declare_parameter('linear_max_check', 10.0)
        self.declare_parameter('angular_max_check', 2.0)

        self.over_count = 0
        self.total_count = 0

    def speed_callback(self, msg):
        """Callback function for the speed_sub subscriber. Reads the Twist message, 
            checks if its over the speed limits, and records"""

        # update the parameters if they've been changed
        linear_limit = self.get_parameter('linear_max_check').get_parameter_value().double_value
        rotation_limit = self.get_parameter('angular_max_check').get_parameter_value().double_value

        # update the count for the total number of messages recieved
        self.total_count += 1

        twist_keys = ["x", "y", "z"]
        for key in twist_keys:
            linear = getattr(msg.linear, key)
            angular = getattr(msg.angular, key)

            # update the count for the number of messages over the speed limits
            # (but only do it once per message)
            if abs(linear) > linear_limit or abs (angular) > rotation_limit:
                self.over_count += 1
                break

    def timer_callback(self):
        self.get_logger().info(f"Recieved {self.total_count} messages, {np.round(self.over_count/self.total_count,2)*100}% over the speed limit")
        self.over_count = 0
        self.total_count = 0


def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the node class
    speed_checker = SpeedChecker()
    # give control over to ROS - spin on the node
    rclpy.spin(speed_checker)
    # shutdown cleanly when cancelled
    rclpy.shutdown()

if __name__ == "__main__":
    main()
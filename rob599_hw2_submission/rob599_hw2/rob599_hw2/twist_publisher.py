#!/usr/bin/env python3

# This node generates random Twist messages and publishes them across
# the "speed_in" topic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class TwistPublisher(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("twist_publisher")
        self.get_logger().info("Started twist publisher")

        # create a publisher
        self.twist_pub = self.create_publisher(Twist, "speed_in", 10)

        # create a timer to publish at a rate
        self.timer = self.create_timer(1, self.timer_callback)

    def generate_twist(self):
        """Generates random values for the twist message within some arbitrary bounds."""
        msg = Twist()
        twist_keys = ["x", "y", "z"]

        for key in twist_keys:
            linear_val = np.random.randint(low=-20, high=20)*np.random.rand()
            angular_val = np.random.randint(low=-6, high=6)*np.random.rand()

            setattr(msg.linear, key, linear_val)
            setattr(msg.angular, key, angular_val)
        
        return msg
    
    def timer_callback(self):
        """Callback function for the timer, generates and publishes random twist messages"""
        msg = self.generate_twist()
        self.twist_pub.publish(msg)
        self.get_logger().info(f"\speed_in: publishing random Twist")


def main(args=None):
    # initializer rclpy
    rclpy.init(args=args)
    # call an instance of the publisher class
    twist_publisher = TwistPublisher()
    # hand control over to ROS
    rclpy.spin(twist_publisher)
    # shutdown cleanly when asked
    rclpy.shutdown()

if __name__ == "__main__":
    main()
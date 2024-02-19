#!/usr/bin/env python3

# This node limits the rotational and linear speed of a robot. It subscribes
# to Twist messages from the "speed_in" topic and publishes modifed Twist
# messages to the "speed_out" topic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from datetime import datetime

class SpeedLimiter(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("speed_limiter")
        self.get_logger().info("Started speed limiter")

        # set node parameters
        self.declare_parameter('linear_max', 10.0)
        self.declare_parameter('angular_max', 2.0)
        self.declare_parameter('with_watchdog', True)
        self.declare_parameter('watchdog_period', 5.0)

        # create a publisher and subscriber
        self.speed_pub = self.create_publisher(Twist, "speed_out", 10)
        self.speed_sub = self.create_subscription(Twist, "speed_in", self.speed_callback, 10)

        # set watchdog timer
        watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().double_value
        self.watchdog_timer = self.create_timer(watchdog_period, self.timer_callback)
        self.watchdog_counter = 0
        
    def timer_callback(self):
        """Callback function for the watchdog timer"""
        # check if watchdog is active
        with_watchdog = self.get_parameter('with_watchdog').get_parameter_value().bool_value
        # grab the watchdog period param (in case it changed) and reset the watchdog timer period
        watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().double_value
        self.watchdog_timer.timer_period_ns = watchdog_period*1e9

        if with_watchdog:
            # if we haven't recieved any messages, publish a zero-velocity twist
            if self.watchdog_counter == 0:
                self.get_logger().info(f"Watchdog triggered: no messages recieved in {watchdog_period}s")
                self.get_logger().info(f"timer period {self.watchdog_timer.timer_period_ns/1e9}")
                msg = Twist()
                self.speed_pub.publish(msg)

        # reset the watchdog counter
        self.watchdog_counter = 0
    
    def speed_callback(self, msg):
        """Callback function for the speed_sub subscriber. Reads the Twist message,
            caps each attribute at a value determined by the linear_ and angular_max
            parameters, and republishes"""
        
        # record that we've recieved a message
        self.watchdog_counter += 1
        self.get_logger().info(f"Speed_in got {msg}")

        linear_limit = self.get_parameter('linear_max').get_parameter_value().double_value
        rotation_limit = self.get_parameter('angular_max').get_parameter_value().double_value

        # cap each attribute of the Twist message 
        msg.linear.x = msg.linear.x if abs(msg.linear.x) < linear_limit else np.sign(msg.linear.x)*linear_limit
        msg.linear.y = msg.linear.y if abs(msg.linear.y) < linear_limit else np.sign(msg.linear.y)*linear_limit
        msg.linear.z = msg.linear.z if abs(msg.linear.z) < linear_limit else np.sign(msg.linear.z)*linear_limit

        msg.angular.x = msg.angular.x if abs(msg.angular.x) < rotation_limit else np.sign(msg.angular.x)*rotation_limit
        msg.angular.y = msg.angular.y if abs(msg.angular.y) < rotation_limit else np.sign(msg.angular.y)*rotation_limit
        msg.angular.z = msg.angular.z if abs(msg.angular.z) < rotation_limit else np.sign(msg.angular.z)*rotation_limit

        self.speed_pub.publish(msg)
        self.get_logger().info(f"Speed_out sent {msg}")

def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the node class
    speed_limiter = SpeedLimiter()
    # give control over to ROS - spin on the node
    rclpy.spin(speed_limiter)
    # shutdown cleanly when cancelled
    rclpy.shutdown()

if __name__ == "__main__":
    main()
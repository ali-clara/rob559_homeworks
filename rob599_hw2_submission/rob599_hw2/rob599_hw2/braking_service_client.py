#!/usr/bin/env python3

# This node calls the apply_breaks service set up in speed_limiter.py

import rclpy
from rclpy.node import Node
from rob599_hw2_msgs.srv import ApplyBrakes

class BrakingServiceClient(Node):
    def __init__(self):
        # initialize the node
        super().__init__('braking_client')
        # initialize the service client
        self.client = self.create_client(ApplyBrakes, "apply_brakes")
        # wait until we have a connection to the server.
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for service to start')

    def send_request(self, braking_status):
        # build the request
        request = ApplyBrakes.Request()
        request.status = braking_status
        # make the service call (asynchronous)
        self.response = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    client = BrakingServiceClient()

    # communicate with the user
    while rclpy.ok():
        brakes_status = input("Set brakes status (on/off/quit) \n")
        if brakes_status == "off":
            client.send_request(False)
            client.get_logger().info("Brakes off")
        elif brakes_status == "quit":
            client.send_request(True)
            client.get_logger().info("Brakes on, quitting service")
            break
        else:
            client.send_request(True)
            client.get_logger().info("Brakes on")
            
    # Shut things down when we're done.
    rclpy.shutdown()

if __name__ == "__main__":
    main()
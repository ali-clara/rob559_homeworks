### Ali Jones
## ROB599 Homework 2

To start the three nodes that emulate publishing, checking, and capping the velocity of a robot, run

    ros2 launch rob599_hw2 speed_launch.py

This launches the nodes in **speed_checker.py**, **speed_limiter.py**, and **twist_publisher.py**. These nodes have the following characteristics:

- "twist_publisher" : publishes random Twist values. Linear values are between -20 and 20m/s, angular values are between -6 and 6m/s
- "speed_checker" : checks if the linear and angular values are over a certain limit. These limits are stored in the parameters *linear_max_check* and *angular_max_check*
- "speed_limiter" : caps the linear and angular values at a certain limit. These limits are stored in the parameters *linear_max* and *angular_max*. By default they are the same as the parameters in "speed_checker". This node also has a watchdog timer with a default period of 5 seconds.

To turn on the brakes (publish a zero-velocity twist message), either call the **apply_brakes** service from the command line or use the interface in **braking_service_client.py**.

Turning on the brakes from the command line:

    ros2 service call /apply_brakes rob599_hw2_msgs/srv/ApplyBrakes "{status: True}"

Using the service client:

    ros2 run rob599_hw2 braking_service_client

---
To start the action for NASA rocket launches (a 100% safe use case of ROS2), run:

    ros2 run rob599_hw2 nasa_action_server

This will start up and wait for a goal sent from the action client. To launch the rocket, run:

    ros2 run rob599_hw2 nasa_action_client <rocket_abort_time>

Where <rocket_abort_time> is an optional integer argument that sets the number of seconds of countdown after which to cancel the launch. This could have also been implemented as a separate service call, but a timer was easier.
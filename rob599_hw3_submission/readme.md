### Ali Jones
## ROB599 Homework 3

Dependencies:

gazebo

    sudo apt install ros-humble-gazebo-*

cartographer

    sudo apt install ros-humble-cartographer

    sudo apt install ros-humble-cartographer-ros

turtlebot

    sudo apt install ros-humble-turtlebot3-gazebo


nav package

    sudo apt install ros-humble-navigation2
    
    sudo apt install ros-humble-nav2-bringup


Running

Drop a marker where the robot is in space, e.g at the "start" position:

    ros2 service call /memorize_position rob599_hw3_msgs/srv/MemorizePosition "{place_name: "start"}"

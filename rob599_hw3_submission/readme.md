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

Launch slam + nav:

Launch localization + nav:

Run the *places* node - please run from the package directory so files get saved in the right place!

    colcon_cd rob599_hw3 && ros2 run rob599_hw3 places

Drop a marker where the robot is in space, e.g at the "start" position:

    ros2 service call /memorize_position rob599_hw3_msgs/srv/MemorizePosition "{place_name: "start"}"

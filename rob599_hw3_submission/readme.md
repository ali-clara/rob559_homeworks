### Ali Jones
## ROB599 Homework 3


### Dependencies:

Gazebo

    sudo apt install ros-humble-gazebo-*

Cartographer

    sudo apt install ros-humble-cartographer

    sudo apt install ros-humble-cartographer-ros

Turtlebot

    sudo apt install ros-humble-turtlebot3-gazebo


Nav2

    sudo apt install ros-humble-navigation2
    
    sudo apt install ros-humble-nav2-bringup


### Running

Launch slam + navigation (part 2):

    ros2 launch rob599_hw3 turtlebot3_slam_launch.py


Launch navigation (part 4):

    ros2 launch rob599_hw3 turtlebot3_nav_launch.py

Run the *places* node (part 5) - please run from the package directory so files get saved in the right place!

    colcon_cd rob599_hw3 && ros2 run rob599_hw3 places

### Service Calls & Additional Functionality

Drop a marker where the robot is in space, e.g at the "start" position (part 5):

    ros2 service call /memorize_position rob599_hw3_msgs/srv/MemorizePosition "{place_name: "start"}"

Clear all markers (part 5):

    ros2 service call /clear_positions rob599_hw3_msgs/srv/ClearPositions "{clear: True}"


Save the recorded position names and poses to a yaml file (part 6):

    ros2 service call /save rob599_hw3_msgs/srv/Save "{filename: "filename.yaml"}"

Load the recorded place names (part 6):

    ros2 service call /load rob599_hw3_msgs/srv/Load "{filename: "filename.yaml"}"

Drive to a location on the map we've previously visited and recorded, e.g back to the "start" position (part 7):

    ros2 run rob599_hw3 goto_action_client "start"


I'm choosing to devote more time to the project instead and turn in this homework without parts 8 and 9 done. Thanks!
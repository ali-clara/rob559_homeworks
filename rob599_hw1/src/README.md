### Ali Jones
## ROB599 Homework 1

To run the **basic wall-stare** with a default stopping distance of 1.0m and a line fit to the wall: 
    
    roslaunch rob599_hw1 wall_stare.launch

- Assumptions:
    - The Fetch gazebo world has been set up as in *roslaunch rob599_basic fetch_world.launch*
    - The Fetch's **arm is not in front of the laser scanner** (I turned gravity off for the wrist in Gazebo and called it good)
- It launches nodes stored in *drive_forward.py*, *trim_laser_scan.py*, and *wall_fit.py*

To run the **service** that controls the distance at which the Fetch stops in front of the nearest obstacle:

    rosrun rob599_hw1 distance_service.py

- Assumptions:
    - The wall_stare launch file has been run
- It brings up an interface in the terminal to ask for user input, and it discards negative numbers or any type that can't be cast to a float

To run the **action** that allows the user to set a stopping distance and provides feedback (how much further the Fetch has to travel) until the goal is reached:

    rosrun rob599_hw1 distance_action.py <desired_stopping_distance>

- Where <desired_stopping_distance> is an optional argument that defaults to 1.0 if not set
- Assumptions:
    - The wall_stare launch file has been run.

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='speed_limiter',
        ),

        launch_ros.actions.Node(
            package='rob599_hw2',
            executable="twist_publisher"
        ),

        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='speed_checker'
        ),
    ])
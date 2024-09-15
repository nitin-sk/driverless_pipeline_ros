import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_detection_package',
            executable='camera_node',
            name='camera_node'
        ),
        launch_ros.actions.Node(
            package='my_detection_package',
            executable='slam_node',
            name='slam_node'
        ),
    ])

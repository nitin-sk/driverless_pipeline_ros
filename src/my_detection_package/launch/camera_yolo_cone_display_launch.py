from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_detection_package',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='my_detection_package',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='my_detection_package',
            executable='cone_detection_node',
            name='cone_detection_node',
            output='screen'
        ),
        Node(
            package='my_detection_package',
            executable='display_node',
            name='display_node',
            output='screen'
        )
    ])


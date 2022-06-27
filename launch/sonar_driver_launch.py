from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar_driver',
            namespace='sonar',
            executable='oculus_driver',
            name='sonar_driver'
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar_driver',
            namespace='sonar',
            executable='OculusDriverNode',
            prefix=['xterm -e gdb -ex=r --args'],
            name='sonar_driver'
        )
    ])

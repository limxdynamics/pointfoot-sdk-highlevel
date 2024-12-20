from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the robot IP parameter
        Node(
            package='pointfoot_highlevel',
            executable='pointfoot_highlevel_node',
            name='pointfoot_highlevel_node',
            output='screen',
            parameters=[{'robot_ip': '10.192.1.2'}],
        ),
    ])
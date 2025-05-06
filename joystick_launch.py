from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_joystick_control',
            executable='f1tenth_joystick_control',
            name='racecar',
        ),

        #
        # Joystick Drivers
        #
        Node(
            package='joy',
            executable='joy_node',
            name='joystick',
        ),
    ])

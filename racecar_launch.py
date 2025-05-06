import os
import launch
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    #
    # VESC stuff (from vesc_driver_node.launch.py)
    #

    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
        )

    print("VESC config file path: ", vesc_config)

    #
    # Hokuyo stuff (from urg_node2.launch.py)
    #

    config_file_path = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether.yaml'
    )

    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    #
    # Launch!
    #

    return LaunchDescription([
        #
        # UB F1Tenth packages
        #

        Node(
            package='f1tenth_gap_follow',
            executable='f1tenth_gap_follow',
            name='racecar'
        ),
        Node(
            package='f1tenth_multiplexer',
            executable='f1tenth_multiplexer',
            name='racecar',
        ),
        # Node(
        #     package='f1tenth_joystick_control',
        #     executable='f1tenth_joystick_control',
        #     name='racecar',
        # ),

        #
        # Joystick Drivers
        #
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joystick',
        # ),

        #
        # VESC
        #

        DeclareLaunchArgument(
            name="vesc_config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration("vesc_config")]
        ),

        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[{
                'speed_to_erpm_gain': 4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -1.2135,
                'steering_angle_to_servo_offset': 0.5304,
            }]
        ),

        #
        # Hokuyo
        #

        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='scan'),
        lifecycle_node,
        urg_node2_node_configure_event_handler,
        urg_node2_node_activate_event_handler,
    ])

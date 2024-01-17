from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_controller2',
            namespace='teleop_twist_controller2',
            executable='teleop_twist_controller2.py',
            name='teleop_twist_controller2',
            parameters=[
                {'device_number': 1},
                {'speed': 0.26},
                {'turn': 1.82},
                {'cmd_vel_topic': '/cmd_vel'},
                {'repeat_rate': 0.0}
            ],
            output='screen',
            emulate_tty=True
        )
    ])

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinematic_model',  
            executable='rosbot_publish_pkg',  
            name='kinematic_model',
            output='screen',
        ),
        Node(
            package='rosbot_control',  
            executable='rosbot_control_pkg',  
            name='wheel_velocities_publisher',
            output='screen',
        ),
    ])

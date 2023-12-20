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
            package='eight_trajectory',  
            executable='eight_trajectory_pkg',  
            name='eight_trajectory',
            output='screen',
        ),
    ])

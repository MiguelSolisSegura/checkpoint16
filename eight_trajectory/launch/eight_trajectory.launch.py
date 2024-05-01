from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
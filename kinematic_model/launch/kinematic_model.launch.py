from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            name='wheel_velocities_publisher',
            output='screen',
            parameters=[{
                'linear_x': 0.5, 
                'linear_y': 0.5,
                'angular_z': 0.5,
                'use_sim_time': True
            }]
        ),
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
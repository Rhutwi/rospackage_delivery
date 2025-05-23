from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='mecanum_navigator',
            name='mecanum_navigator',
            output='screen',
            # remap /goal_pose, /odom, /cmd_vel as needed
        ),
    ])

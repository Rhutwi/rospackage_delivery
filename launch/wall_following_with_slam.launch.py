from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Delayed SLAM Toolbox launch (3 seconds)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'scan_queue_size': 1000}  # 🆕 Increase queue size to prevent drops
                    ],
                    remappings=[('/scan', '/scan')]  # Remap if needed
                )
            ]
        ),

        # Wall Following Node (starts immediately)
        Node(
            package='wall_following',
            executable='wall_follow_node',
            name='wall_follow_node',
            output='screen',
        ),
    ])

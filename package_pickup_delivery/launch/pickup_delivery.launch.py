# launch/pickup_delivery.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ─── 0a) map → odom (if slam_toolbox isn’t visible on host TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '0','0','0',      # x y z
                '0','0','0',      # roll pitch yaw
                'map','odom'      # parent_frame child_frame
            ],
            output='screen',
        ),

        # ─── 0b) odom → base_link (if your robot driver isn’t visible on host TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '0','0','0',      # x y z
                '0','0','0',      # roll pitch yaw
                'odom','base_link'
            ],
            output='screen',
        ),

        # ─── 0c) base_link → camera_link (camera mounting offset)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0.0','0.0','0.10',  # adjust Z to your camera height (m)
                '0','0','0',         # roll pitch yaw
                'base_link','camera_link'
            ],
            output='screen',
        ),

        # ─── 1) ArUco detector (publishes /aruco/pose & /aruco/id)
        Node(
            package='package_pickup_delivery',
            executable='aruco_detector',
            name='aruco_detector',
            # point its parameters at the OAK-D topics:
            parameters=[{
                'image_topic':       '/oak/rgb/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'marker_size':        0.03,
                'aruco_dictionary':   'DICT_4X4_50',
            }],
            # in case internal code still uses "/camera/..." names:
            remappings=[
                ('/camera/image_raw',   '/oak/rgb/image_raw'),
                ('/camera/camera_info', '/oak/rgb/camera_info'),
            ],
            output='screen',
        ),

        # ─── 2) TF pose transformer (re-publishes → /aruco/map_pose in `map` frame)
        Node(
            package='package_pickup_delivery',
            executable='tf_pose_transformer',
            name='tf_pose_transformer',
            parameters=[{
                'input_pose_topic':  '/aruco/pose',
                'input_id_topic':    '/aruco/id',
                'output_pose_topic': '/aruco/map_pose',
                'target_frame':      'map',
            }],
            output='screen',
        ),

        # ─── 3) Task planner (listens on /aruco/id & TF → publishes /navigate_to_pose and /task_command)
        Node(
            package='package_pickup_delivery',
            executable='task_planner',
            name='task_planner',
            output='screen',
        ),

        # ─── 4) (Optional) marker_touch_verifier, RViz, etc.
        # Node(
        #     package='package_pickup_delivery',
        #     executable='marker_touch_verifier',
        #     name='marker_touch_verifier',
        #     parameters=[{
        #         'marker_topic':   '/aruco/map_pose',
        #         'pointer_frame':  'pointer_tip',
        #         'target_frame':   'map',
        #         'tolerance_m':    0.05,
        #     }],
        #     output='screen',
        # ),
        #
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d','/path/to/your/config.rviz'],
        #     output='screen',
        # ),
    ])



# # launch/pickup_delivery.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([

#         # ─── 0a) map → odom
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='map_to_odom',
#             arguments=['0','0','0','0','0','0','map','odom'],
#             output='screen',
#         ),

#         # ─── 0b) odom → base_link
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='odom_to_base',
#             arguments=['0','0','0','0','0','0','odom','base_link'],
#             output='screen',
#         ),

#         # ─── 0c) base_link → camera_link (camera height=0.10 m)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='base_to_camera',
#             arguments=['0.0','0.0','0.10','0','0','0','base_link','camera_link'],
#             output='screen',
#         ),

#         # ─── 1) ArUco detector
#         Node(
#             package='package_pickup_delivery',
#             executable='aruco_detector',
#             name='aruco_detector',
#             parameters=[{
#                 'image_topic':       '/camera/image_raw',
#                 'camera_info_topic': '/camera/camera_info',
#                 'marker_size':       0.03,
#                 'aruco_dictionary':  'DICT_4X4_50',
#             }],
#             output='screen',
#         ),

#         # ─── 2) TF pose transformer → /aruco/map_pose
#         Node(
#             package='package_pickup_delivery',
#             executable='tf_pose_transformer',
#             name='tf_pose_transformer',
#             parameters=[{
#                 'input_pose_topic':  '/aruco/pose',
#                 'input_id_topic':    '/aruco/id',
#                 'output_pose_topic': '/aruco/map_pose',
#                 'target_frame':      'map',
#             }],
#             output='screen',
#         ),

#         # ─── 3) Task planner
#         Node(
#             package='package_pickup_delivery',
#             executable='task_planner',
#             name='task_planner',
#             output='screen',
#         ),
#     ])


# # launch/pickup_delivery.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([

#         # ─── 0a) map → odom
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='map_to_odom',
#             arguments=['0','0','0','0','0','0','map','odom'],
#             output='screen',
#         ),

#         # ─── 0b) odom → base_link
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='odom_to_base',
#             arguments=['0','0','0','0','0','0','odom','base_link'],
#             output='screen',
#         ),

#         # ─── 0c) base_link → camera_link (camera height=0.10 m)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='base_to_camera',
#             arguments=['0.0','0.0','0.10','0','0','0','base_link','camera_link'],
#             output='screen',
#         ),

#         # ─── 1) ArUco detector
#         Node(
#             package='package_pickup_delivery',
#             executable='aruco_detector',
#             name='aruco_detector',
#             parameters=[{
#                 'image_topic':       '/camera/image_raw',
#                 'camera_info_topic': '/camera/camera_info',
#                 'marker_size':       0.03,
#                 'aruco_dictionary':  'DICT_4X4_50',
#             }],
#             output='screen',
#         ),

#         # ─── 2) TF pose transformer → /aruco/map_pose
#         Node(
#             package='package_pickup_delivery',
#             executable='tf_pose_transformer',
#             name='tf_pose_transformer',
#             parameters=[{
#                 'input_pose_topic':  '/aruco/pose',
#                 'input_id_topic':    '/aruco/id',
#                 'output_pose_topic': '/aruco/map_pose',
#                 'target_frame':      'map',
#             }],
#             output='screen',
#         ),

#         # ─── 3) Task planner
#         Node(
#             package='package_pickup_delivery',
#             executable='task_planner',
#             name='task_planner',
#             output='screen',
#         ),
#     ])



# # launch/pickup_delivery.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([

#         # ─── 0a) map → odom (if slam_toolbox isn’t visible on host TF)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='map_to_odom',
#             arguments=[
#                 '0','0','0',      # x y z
#                 '0','0','0',      # roll pitch yaw
#                 'map','odom'      # parent_frame child_frame
#             ],
#             output='screen',
#         ),

#         # ─── 0b) odom → base_link (if your robot driver isn’t visible on host TF)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='odom_to_base',
#             arguments=[
#                 '0','0','0',      # x y z
#                 '0','0','0',      # roll pitch yaw
#                 'odom','base_link'
#             ],
#             output='screen',
#         ),

#         # ─── 0c) base_link → camera_link (camera mounting offset)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='base_to_camera',
#             arguments=[
#                 '0.0','0.0','0.10',  # adjust Z to your camera height (m)
#                 '0','0','0',         # roll pitch yaw
#                 'base_link','camera_link'
#             ],
#             output='screen',
#         ),

#         # ─── 1) ArUco detector (publishes /aruco/pose & /aruco/id)
#         Node(
#             package='package_pickup_delivery',
#             executable='aruco_detector',
#             name='aruco_detector',
#             parameters=[{
#                 'image_topic':        '/camera/image_raw',
#                 'camera_info_topic':  '/camera/camera_info',
#                 'marker_size':        0.03,
#                 'aruco_dictionary':   'DICT_4X4_50',
#             }],
#             output='screen',
#         ),

#         # ─── 2) TF pose transformer (re‐publishes → /aruco/map_pose in `map` frame)
#         Node(
#             package='package_pickup_delivery',
#             executable='tf_pose_transformer',
#             name='tf_pose_transformer',
#             parameters=[{
#                 'input_pose_topic':   '/aruco/pose',
#                 'input_id_topic':     '/aruco/id',
#                 'output_pose_topic':  '/aruco/map_pose',
#                 'target_frame':       'map',
#             }],
#             output='screen',
#         ),

#         # ─── 3) Task planner (listens on /aruco/id & TF → publishes /navigate_to_pose and /task_command)
#         Node(
#             package='package_pickup_delivery',
#             executable='task_planner',
#             name='task_planner',
#             output='screen',
#         ),

        # ─── 4) (Optional) touch verifier, rviz2, etc.
        # Node(
        #     package='package_pickup_delivery',
        #     executable='marker_touch_verifier',
        #     name='marker_touch_verifier',
        #     parameters=[{ 'marker_topic': '/aruco/map_pose',
        #                   'pointer_frame': 'pointer_tip',
        #                   'target_frame': 'map',
        #                   'tolerance_m': 0.05 }],
        #     output='screen',
        # ),
        #
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d','path/to/your/config.rviz'],
        #     output='screen',
        # ),
    # ])


# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#       # 1) static map → camera_link
#       Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0','0','0','0','0','0','map','camera_link'],
#       ),

#       # 2) ArUco detector
#       Node(
#         package='package_pickup_delivery',
#         executable='aruco_detector',
#         parameters=[{
#           'image_topic': '/camera/image_raw',
#           'camera_info_topic': '/camera/camera_info',
#           'marker_size': 0.03,
#           'aruco_dictionary': 'DICT_4X4_50',
#         }],
#       ),

#       3) TF-based pose transformer
#       Node(
#         package='package_pickup_delivery',
#         executable='tf_pose_transformer',
#         parameters=[{
#           'input_pose_topic': '/aruco/pose',
#           'input_id_topic': '/aruco/id',
#           'output_pose_topic': '/aruco/map_pose',
#           'target_frame': 'map',
#         }],
#       ),

#       # 4) Touch verifier (if used) …
#       # 5) Task planner
#       Node(
#         package='package_pickup_delivery',
#         executable='task_planner',
#         output='screen'
#       ),
#     ])
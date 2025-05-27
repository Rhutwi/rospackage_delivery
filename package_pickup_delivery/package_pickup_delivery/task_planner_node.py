
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException

class TaskPlannerNode(Node):
    """
    Listens to /aruco/id (Int32).
    On each new marker ID:
      • Look up the latest TF from 'camera_link' -> 'marker_<ID>'
      • Decide pickup vs delivery based on a simple map
      • Publish a PoseStamped goal on /navigate_to_pose
      • Publish a String command on /task_command
    """

    def __init__(self):
        super().__init__('task_planner_node')

        # Pickup → Delivery marker ID mapping
        self.marker_pairs = {
            0: 1,
            2: 3,
            4: 5,
            6: 7,
            8: 9,
            10: 11,
            12: 13,
            14: 15,
            16: 17,
            18: 19,
            20: 21,
            22: 23,
            24: 25,
            26: 27,
            28: 29,
        }

        self.carrying = False
        self.expected_delivery = None

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber to marker ID topic
        self.create_subscription(Int32, '/aruco/id', self.on_marker_seen, 10)

        # Publishers for goal and task commands
        self.goal_pub = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.task_pub = self.create_publisher(String, '/task_command', 10)

        self.get_logger().info("🚦 TaskPlannerNode started, waiting on /aruco/id…")

    def on_marker_seen(self, msg: Int32):
        mid = msg.data
        self.get_logger().info(f"🔍 Seen marker {mid}")

        # Try to lookup TF from camera_link → marker_<ID>
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'camera_link', f'marker_{mid}', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"❗ TF not ready for marker_{mid}: {e}")
            return

        t = tf_stamped.transform.translation
        x, y = t.x, t.y

        # Check if it's a pickup marker
        if mid in self.marker_pairs:
            self.get_logger().info(f"📦 Pickup marker {mid} @ ({x:.2f}, {y:.2f})")
            self.send_goal(x, y, tf_stamped.header.stamp)
            self.send_task(f"pickup:{mid}")
            self.expected_delivery = self.marker_pairs[mid]
            self.carrying = True

        # Check if it's a delivery marker (even without a pickup)
        elif mid in self.marker_pairs.values():
            self.get_logger().info(f"📬 Delivery marker {mid} @ ({x:.2f}, {y:.2f})")
            self.send_goal(x, y, tf_stamped.header.stamp)
            self.send_task(f"delivery:{mid}")
            self.carrying = False
            self.expected_delivery = None

        else:
            self.get_logger().warn(f"⚠ Marker {mid} is not recognized as pickup or delivery")

    def send_goal(self, x: float, y: float, stamp):
        goal = PoseStamped()
        goal.header.frame_id = 'camera_link'
        goal.header.stamp = stamp
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # Facing forward

        self.goal_pub.publish(goal)
        self.get_logger().info(f"🚀 Goal sent → x={x:.2f}, y={y:.2f}")

    def send_task(self, cmd: str):
        m = String()
        m.data = cmd
        self.task_pub.publish(m)
        self.get_logger().info(f"📤 Task sent → '{cmd}'")


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import PoseStamped
# import tf2_ros
# from tf2_ros import TransformException

# class TaskPlannerNode(Node):
#     """
#     Listens to /aruco/id (Int32).
#     On each new marker ID:
#       • Look up the latest TF from 'camera_link' -> 'marker_<ID>'
#       • Decide pickup vs delivery based on a simple map
#       • Publish a PoseStamped goal on /navigate_to_pose
#       • Publish a String command on /task_command
#     """

#     def __init__(self):
#         super().__init__('task_planner_node')

#         # 1) Define your pickup->delivery mapping here
#         self.marker_pairs = {
#             0: 1,
#             2: 3,
#             4: 5,
#         }
#         self.carrying = False
#         self.expected_delivery = None

#         # 2) TF buffer & listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # 3) Subscribe to the detector's ID topic
#         self.create_subscription(
#             Int32, '/aruco/id', self.on_marker_seen, 10)

#         # 4) Publishers for navigation + task command
#         self.goal_pub = self.create_publisher(
#             PoseStamped, '/navigate_to_pose', 10)
#         self.task_pub = self.create_publisher(
#             String, '/task_command', 10)

#         self.get_logger().info("🚦 TaskPlannerNode started, waiting on /aruco/id…")

#     def on_marker_seen(self, msg: Int32):
#         mid = msg.data
#         self.get_logger().info(f"🔍 Seen marker {mid}")

#         # Try to lookup the latest transform from camera_link -> marker_<ID>
#         try:
#             tf_stamped = self.tf_buffer.lookup_transform(
#                 'camera_link',           # target frame
#                 f'marker_{mid}',         # source frame
#                 rclpy.time.Time())       # latest
#         except TransformException as e:
#             self.get_logger().warn(f"❗ TF not ready for marker_{mid}: {e}")
#             return

#         # Extract x,y from the transform
#         t = tf_stamped.transform.translation
#         x, y = t.x, t.y

#         # Decide pickup or delivery
#         if not self.carrying:
#             if mid in self.marker_pairs:
#                 self.get_logger().info(f"📦 Pickup marker {mid} @ ({x:.2f},{y:.2f})")
#                 self.send_goal(x, y, tf_stamped.header.stamp)
#                 self.send_task(f"pickup:{mid}")
#                 self.expected_delivery = self.marker_pairs[mid]
#                 self.carrying = True
#             else:
#                 self.get_logger().warn(f"⚠️ {mid} is not a pickup ID")
#         else:
#             if mid == self.expected_delivery:
#                 self.get_logger().info(f"📬 Delivery marker {mid} @ ({x:.2f},{y:.2f})")
#                 self.send_goal(x, y, tf_stamped.header.stamp)
#                 self.send_task(f"delivery:{mid}")
#                 self.carrying = False
#                 self.expected_delivery = None
#             else:
#                 self.get_logger().warn(
#                     f"❗ Got {mid} but expecting delivery {self.expected_delivery}"
#                 )

#     def send_goal(self, x: float, y: float, stamp):
#         goal = PoseStamped()
#         goal.header.frame_id = 'camera_link'
#         goal.header.stamp = stamp
#         goal.pose.position.x = x
#         goal.pose.position.y = y
#         goal.pose.position.z = 0.0
#         goal.pose.orientation.w = 1.0

#         self.goal_pub.publish(goal)
#         self.get_logger().info(f"🚀 Goal sent → x={x:.2f}, y={y:.2f}")

#     def send_task(self, cmd: str):
#         m = String()
#         m.data = cmd
#         self.task_pub.publish(m)
#         self.get_logger().info(f"📤 Task sent → '{cmd}'")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TaskPlannerNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, PointStamped
# from std_msgs.msg import String
# import time

# class TaskPlannerNode(Node):
#     def __init__(self):
#         super().__init__('task_planner_node')

#         # Subscribers
#         self.create_subscription(String, '/detected_markers', self.marker_callback, 10)
#         self.create_subscription(PointStamped, '/detected_marker_pose', self.pose_callback, 10)

#         # Publishers
#         self.goal_pub = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
#         self.task_pub = self.create_publisher(String, '/task_command', 10)

#         # Pair mapping: pickup_id -> delivery_id
#         self.marker_pairs = {
#             '0': '1',
#             '2': '3',
#             '4': '5'
#         }

#         # State
#         self.package_carried = False
#         self.last_marker_id = None
#         self.last_marker_time = 0
#         self.cooldown_seconds = 5
#         self.current_marker_pose = None
#         self.expected_delivery_id = None

#         self.get_logger().info("🚦 Task Planner Node (ArUco Pair Version) Started.")

#     def marker_callback(self, msg):
#         marker_id = msg.data.strip()
#         current_time = time.time()

#         # Cooldown check
#         if marker_id == self.last_marker_id and (current_time - self.last_marker_time) < self.cooldown_seconds:
#             self.get_logger().info(f"⏳ Marker {marker_id} ignored (cooldown active)")
#             return

#         self.last_marker_id = marker_id
#         self.last_marker_time = current_time

#         if self.current_marker_pose is None:
#             self.get_logger().warn("⚠️ Marker pose not received yet!")
#             return

#         x = self.current_marker_pose.point.x
#         y = self.current_marker_pose.point.y

#         # Main logic
#         if not self.package_carried:
#             if marker_id in self.marker_pairs:
#                 self.get_logger().info(f"📦 Pickup marker {marker_id} detected at ({x:.2f}, {y:.2f})")
#                 self.send_goal(x, y, 0.0)
#                 self.send_task_command("pickup", marker_id)
#                 self.expected_delivery_id = self.marker_pairs[marker_id]
#                 self.package_carried = True
#             else:
#                 self.get_logger().warn(f"❌ Marker {marker_id} is not a pickup marker!")
#         else:
#             if marker_id == self.expected_delivery_id:
#                 self.get_logger().info(f"📬 Delivery marker {marker_id} detected at ({x:.2f}, {y:.2f})")
#                 self.send_goal(x, y, 0.0)
#                 self.send_task_command("delivery", marker_id)
#                 self.package_carried = False
#                 self.expected_delivery_id = None
#             else:
#                 self.get_logger().warn(f"⚠️ Marker {marker_id} is not the expected delivery marker (expected {self.expected_delivery_id})")

#     def pose_callback(self, msg):
#         self.current_marker_pose = msg

#     def send_goal(self, x, y, yaw):
#         goal = PoseStamped()
#         goal.header.frame_id = "map"
#         goal.header.stamp = self.get_clock().now().to_msg()
#         goal.pose.position.x = x
#         goal.pose.position.y = y
#         goal.pose.orientation.w = 1.0
#         self.goal_pub.publish(goal)
#         self.get_logger().info(f"🚀 Goal sent: x={x:.2f}, y={y:.2f}")

#     def send_task_command(self, task_type, marker_id):
#         task = String()
#         task.data = f"{task_type}:{marker_id}"
#         self.task_pub.publish(task)
#         self.get_logger().info(f"📤 Task command sent: {task.data}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TaskPlannerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

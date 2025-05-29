
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


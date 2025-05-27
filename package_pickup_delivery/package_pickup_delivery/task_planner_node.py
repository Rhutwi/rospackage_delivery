#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class TaskPlannerNode(Node):
    """
    Listens to /aruco/id.
    For pickup markers, it sends a goal and pickup command.
    For delivery markers, it sends a goal and delivery command.
    """

    def __init__(self):
        super().__init__('task_planner_node')

        # Manually written marker ID pairs: pickup → delivery
        self.marker_pairs = {
            0: 1, 2: 3, 4: 5, 6: 7, 8: 9,
            10: 11, 12: 13, 14: 15, 16: 17,
            18: 19, 20: 21, 22: 23, 24: 25,
            26: 27, 28: 29, 30: 31, 32: 33,
            34: 35, 36: 37, 38: 39, 40: 41,
            42: 43, 44: 45, 46: 47, 48: 49,
            50: 51, 52: 53, 54: 55, 56: 57,
            58: 59, 60: 61, 62: 63, 64: 65,
            66: 67, 68: 69, 70: 71, 72: 73,
            74: 75, 76: 77, 78: 79, 80: 81,
            82: 83, 84: 85, 86: 87, 88: 89,
            90: 91, 92: 93, 94: 95, 96: 97,
            98: 99
        }

        self.carrying = False
        self.expected_delivery = None
        self.last_marker_id = None
        self.waiting_for_nav = False

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Int32, '/aruco/id', self.on_marker_seen, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.task_pub = self.create_publisher(String, '/task_command', 10)

        self.get_logger().info("🚦 TaskPlannerNode ready.")

    def on_marker_seen(self, msg: Int32):
        mid = msg.data

        # Debounce duplicate detections
        if mid == self.last_marker_id:
            return

        if self.waiting_for_nav:
            self.get_logger().warn("🛑 Navigation in progress, ignoring marker.")
            return

        self.last_marker_id = mid

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'camera_link', f'marker_{mid}', Time())
        except TransformException as e:
            self.get_logger().warn(f"❗ TF error for marker_{mid}: {e}")
            return

        x = tf_stamped.transform.translation.x
        y = tf_stamped.transform.translation.y
        stamp = tf_stamped.header.stamp

        if mid in self.marker_pairs:
            if self.carrying:
                self.get_logger().warn("⚠️ Already carrying a package, can't pickup again.")
                return

            self.send_goal(x, y, stamp)
            self.send_task(f"pickup:{mid}")
            self.expected_delivery = self.marker_pairs[mid]
            self.carrying = True
            self.waiting_for_nav = True
            self.get_logger().info(f"📦 Pickup task for marker {mid}")

        elif mid == self.expected_delivery:
            self.send_goal(x, y, stamp)
            self.send_task(f"delivery:{mid}")
            self.expected_delivery = None
            self.carrying = False
            self.waiting_for_nav = True
            self.get_logger().info(f"📬 Delivery task for marker {mid}")

        else:
            self.get_logger().warn(f"❓ Marker {mid} is not expected or not in known pairs.")

    def send_goal(self, x, y, stamp):
        goal = PoseStamped()
        goal.header.frame_id = 'camera_link'
        goal.header.stamp = stamp
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"🚀 Goal sent: x={x:.2f}, y={y:.2f}")

    def send_task(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.task_pub.publish(msg)
        self.get_logger().info(f"📤 Task command sent: '{cmd}'")


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

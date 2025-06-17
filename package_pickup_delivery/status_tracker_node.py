import rclpy
from std_msgs.msg import Int32
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import os

class StatusTracker(Node):
    def __init__(self):
        super().__init__('status_tracker_node')

        # Subscribers
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)
        self.create_subscription(PoseStamped, '/aruco/map_pose', self.marker_callback, 10)
        self.create_subscription(Bool, '/pointer/aligned', self.pointer_callback, 10)
        self.create_subscription(Int32, '/aruco/id', self.id_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/package_status', 10)
        self.complete_pub = self.create_publisher(Bool, '/delivery_complete', 10)
        self.log_pub = self.create_publisher(String, '/log_events', 10)

        # State
        self.expected_marker_id = None
        self.expected_task_type = None
        self.last_seen_marker_id = None
        self.current_marker_pose = None

        # Optional: log file path
        log_dir = os.path.expanduser("~/rosbot_logs")
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(os.path.join(log_dir, "task_log.txt"), "a")

        self.get_logger().info("✅ Status Tracker Node (ROS 2) is running...")

    def task_status_callback(self, msg):
        task_text = msg.data.lower()
        self.get_logger().info(f"[TaskStatus] {task_text}")
        self.write_log(f"[TaskStatus] {task_text}")

        if "picking up from marker" in task_text:
            self.expected_task_type = "pickup"
            self.expected_marker_id = int(task_text.split("marker")[1].strip())
        elif "delivering to marker" in task_text:
            self.expected_task_type = "delivery"
            self.expected_marker_id = int(task_text.split("marker")[1].strip())
        else:
            self.expected_marker_id = None
            self.expected_task_type = None

    def id_callback(self, msg: Int32):
        self._last_aruco_id = msg.data

    def marker_callback(self, msg: PoseStamped):
        # Only process if we have just seen an ID
        if self._last_aruco_id is None:
            return
        self.last_seen_marker_id = self._last_aruco_id
        self.current_marker_pose   = msg.pose
        # reset for next
        self._last_aruco_id = None


    def pointer_callback(self, msg):
        self.get_logger().info(f"[DEBUG] pointer/aligned = {msg.data}, expected_marker_id = {self.expected_marker_id}, last_seen = {self.last_seen_marker_id}")
        if msg.data and self.expected_marker_id is not None and self.current_marker_pose is not None:
            if self.last_seen_marker_id == self.expected_marker_id:
                self.log_task_completion()

    def log_task_completion(self):
        now = datetime.now()
        time_str = now.strftime('%Y-%m-%d %H:%M:%S')
        status_str = f"{self.expected_task_type.capitalize()} completed for Marker {self.expected_marker_id} at {time_str}"

        self.get_logger().info(f"[STATUS] {status_str}")
        self.status_pub.publish(String(data=status_str))
        self.complete_pub.publish(Bool(data=True))
        self.log_pub.publish(String(data=status_str))
        self.write_log(f"[STATUS] {status_str}")

        # Reset for next task
        self.expected_marker_id = None
        self.expected_task_type = None
        self.current_marker_pose = None
        self.last_seen_marker_id = None

    def write_log(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_file.write(f"[{timestamp}] {message}\n")
        self.log_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = StatusTracker()
    try:
        rclpy.spin(node)
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from status_tracker_node.msg import Task, DeliveryStatus

class StatusTrackerNode(Node):
    def __init__(self):
        super().__init__('status_tracker_node')
        self.carried_packages = set()
        self.delivered_packages = set()
        self.current_task = None

        self.create_subscription(Task, '/task_command', self.task_callback, 10)
        self.create_subscription(Task, '/nav_status', self.nav_status_callback, 10)
        self.status_pub = self.create_publisher(DeliveryStatus, '/package_status', 10)

        self.get_logger().info('Status Tracker Node is running.')

    def task_callback(self, msg):
        self.current_task = msg
        self.get_logger().info(f'Received task: {msg.task_type} {msg.marker_id}')

    def nav_status_callback(self, msg):
        if not self.current_task or msg.marker_id != self.current_task.marker_id:
            return

        if msg.task_type == 'ARRIVED':
            marker_id = self.current_task.marker_id
            if self.current_task.task_type == 'pickup':
                self.carried_packages.add(marker_id)
                self.get_logger().info(f"Picked up package {marker_id}")
            elif self.current_task.task_type == 'delivery':
                if marker_id in self.carried_packages:
                    self.carried_packages.remove(marker_id)
                    self.delivered_packages.add(marker_id)
                    self.get_logger().info(f"Delivered package {marker_id}")
                else:
                    self.get_logger().warn(f"Tried to deliver uncarried package {marker_id}")

            self.publish_status()
            self.current_task = None

    def publish_status(self):
        msg = DeliveryStatus()
        msg.carried_packages = list(self.carried_packages)
        msg.delivered_packages = list(self.delivered_packages)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StatusTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

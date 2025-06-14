#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')

        self.marker_sub = self.create_subscription(Int32, '/aruco/id', self.marker_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco/map_pose', self.pose_cb, 10)
        self.cmd_sub = self.create_subscription(String, '/task_planner/control', self.control_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_pose = None
        self.current_id = None
        self.is_servoing = False

        self.get_logger().info("🚦 TaskPlannerNode started, waiting on /aruco/id…")

    def control_cb(self, msg: String):
        if msg.data == 'visual_servo_active':
            self.is_servoing = True
            self.get_logger().info("⚠️  Visual servoing active. Pausing planning.")
        elif msg.data == 'visual_servo_done':
            self.is_servoing = False
            self.get_logger().info("✅ Visual servoing done. Resuming planning.")

    def marker_cb(self, msg: Int32):
        self.current_id = msg.data
        self.get_logger().info(f"🔍 Seen marker {self.current_id}")

    def pose_cb(self, msg: PoseStamped):
        if self.is_servoing:
            return
        if self.current_id is None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        self.get_logger().info(f"📦 Pickup marker {self.current_id} @ ({x:.2f}, {y:.2f})")
        self.send_goal(x, y)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # default orientation

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"🚀 Goal sent → x={x:.2f}, y={y:.2f}")


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

# ✅ visual_servo_pointer_node.py (UPDATED for refined reactive behavior)

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class VisualServoPointer(Node):
    def __init__(self):
        super().__init__('visual_servo_pointer')

        self.declare_parameter('pose_topic', '/aruco/pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('xy_gain', 0.7)
        self.declare_parameter('stop_tolerance', 0.001)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.xy_gain = self.get_parameter('xy_gain').value
        self.stop_tolerance = self.get_parameter('stop_tolerance').value

        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(f"🌀 Visual Servo on topic: {self.pose_topic}")

    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        distance = math.sqrt(x**2 + y**2)

        if distance < self.stop_tolerance:
            self.get_logger().info("🚩 Stopping: within tolerance.")
            self.cmd_pub.publish(Twist())
            return

        # Basic proportional controller
        twist = Twist()
        twist.linear.x = self.xy_gain * x
        twist.linear.y = self.xy_gain * y
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"🔄 Servo move x={x:.3f}, y={y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoPointer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

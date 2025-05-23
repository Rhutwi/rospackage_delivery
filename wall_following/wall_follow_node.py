#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from math import sqrt
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info("Wall Follower FSM Node started")

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/start_wall_follow', self.start_callback, 10)
        self.create_subscription(Bool, '/hazard_detected', self.marker_callback, 10)
        self.create_subscription(Bool, '/stop_wall_follow', self.stop_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Parameters and State
        self.regions = {'right': float('inf'), 'front': float('inf'), 'left': float('inf')}
        self.state = 0
        self.start_position = None
        self.current_position = None
        self.loop_closure_triggered = False
        self.right_wall_lost = False
        self.corner_turn_phase = 0
        self.forward_after_right_wall_start = None
        self.active = False
        self.marker_pause = False
        self.marker_pause_start = None
        self.target_distance = 0.5
        self.turning_radius = 0.18
        self.stopped = False

    def update_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def start_callback(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info("Start marker received — wall following activated")
            self.update_status("Start received: Beginning wall-following")

    def marker_callback(self, msg):
        if msg.data and not self.marker_pause:
            self.marker_pause = True
            self.marker_pause_start = time.time()
            self.get_logger().info("Hazard marker seen — pausing and spinning")
            self.update_status("Marker detected: spinning to improve detection")

    def stop_callback(self, msg):
        if msg.data:
            self.stopped = True
            self.active = False
            self.get_logger().warn("Stop signal received — stopping wall follower")
            self.update_status("Wall-following stopped")
            self.cmd_pub.publish(Twist())

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f"Start position recorded at: {self.start_position}")
        elif not self.loop_closure_triggered:
            dist = self.euclidean_distance(self.start_position, self.current_position)
            if dist < 0.3:
                self.loop_closure_triggered = True
                self.get_logger().info("Loop closure detected — re-entering find wall mode")
                self.change_state(0)
                self.update_status("Loop closure — Resetting to wall search")

    def euclidean_distance(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def scan_callback(self, msg):
        ranges = msg.ranges
        def clamp(val, min_val, max_val):
            return max(min(val, max_val), min_val)

        def safe_min(r_list):
            valid = [r for r in r_list if 0.05 < r < 3.0]
            return min(valid) if valid else float('inf')

        self.regions = {
            'right': clamp(safe_min(ranges[75:105]), 0.0, 3.0),
            'front': clamp(safe_min(ranges[0:15] + ranges[-15:]), 0.0, 3.0),
            'left': clamp(safe_min(ranges[255:285]), 0.0, 3.0)
        }

    def change_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"State changed from {self.state} to {new_state}")
            self.state = new_state

    def control_loop(self):
        if self.stopped:
            return

        if not self.active:
            self.update_status("Idle: Waiting for Start Marker")
            return

        if self.marker_pause:
            elapsed = time.time() - self.marker_pause_start
            if elapsed < 3.5:
                msg = Twist()
                msg.angular.z = 0.6
                self.cmd_pub.publish(msg)
                return
            else:
                self.marker_pause = False
                self.update_status("Spin complete — Resuming exploration")

        msg = Twist()
        d = self.target_distance
        front = self.regions['front']
        right = self.regions['right']
        left = self.regions['left']

        if self.state == 0:
            if front < 0.7:
                self.change_state(1)
            elif right < d - 0.1:
                self.change_state(2)

        elif self.state == 1:
            if front > 0.7:
                self.change_state(2)

        elif self.state == 2:
            if front < 0.4:
                self.change_state(1)
            elif front > 0.7 and right > 1.0 and left < 0.6:
                self.get_logger().info("Inside corner detected — turning early")
                self.change_state(1)
            elif right > d + 0.1:
                if right < 1.5:
                    self.get_logger().info("Right wall turning — staying in follow mode")
                elif not self.right_wall_lost:
                    self.get_logger().info("True wall loss — switching to find wall")
                    self.change_state(0)

        if self.state == 0:
            msg.linear.x = 0.15
            msg.angular.z = -0.25
            self.update_status("Searching for wall")

        elif self.state == 1:
            msg.linear.x = 0.12
            msg.angular.z = 0.45
            self.update_status("Turning left")

        elif self.state == 2:
            error = right - d
            msg.linear.x = 0.18
            msg.angular.z = -1.2 * error
            msg.angular.z = max(min(msg.angular.z, 0.6), -0.6)
            self.update_status(f"Following wall | Error: {error:.2f}")

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting FSM WallFollower")
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

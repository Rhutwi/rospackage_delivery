#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int32
from math import sqrt, atan2, pi, asin
import time

def euler_from_quaternion(quaternion):
    """Convert quaternion (x, y, z, w) to (roll, pitch, yaw)"""
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info("Wall Follower FSM Node started")

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)

        # Auto-start enabled
        self.active = True
        self.get_logger().info("Auto-start: wall following activated")
        self.update_status("Auto-start received: Beginning wall-following")

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Bool, '/start_wall_follow', self.start_callback, 10)
        self.create_subscription(Bool, '/hazard_detected', self.marker_callback, 10)
        self.create_subscription(Bool, '/stop_wall_follow', self.stop_callback, 10)
        self.create_subscription(Bool, '/go_to_pose', self.go_to_pose_callback, 10)
        # --- ArUco marker integration ---
        self.create_subscription(PoseStamped, '/aruco/pose', self.aruco_pose_callback, 10)
        self.create_subscription(Int32, '/aruco/id', self.aruco_id_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Parameters and State
        self.regions = {'right': float('inf'), 'front': float('inf'), 'left': float('inf')}
        self.state = 0
        self.start_position = None
        self.current_position = None
        self.current_yaw = 0.0
        self.loop_closure_triggered = False
        self.right_wall_lost = False
        self.corner_turn_phase = 0
        self.forward_after_right_wall_start = None
        self.marker_pause = False
        self.marker_pause_start = None
        self.target_distance = 0.6   # Relaxed: not too close to wall!
        self.min_dist = 0.3          # Do NOT let it get closer than this
        self.turning_radius = 0.18
        self.stopped = False
        self.turning_start_time = None  # For smart turn timeout

        # Go to pose mode
        self.goto_active = False
        self.return_pose = (0.05, 0.09, 0.0)  # (x, y, theta), will be overwritten by ArUco logic

        # ArUco memory and detection
        self.seen_markers = {}   # id: (x, y, yaw)
        self.last_marker_id = None
        self.last_marker_pose = None

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

    def go_to_pose_callback(self, msg):
        if msg.data and not self.goto_active:
            self.get_logger().info("Go To Pose trigger received! Returning to base...")
            self.state = 10  # special state for 'go to pose'
            self.goto_active = True
            self.update_status("Switching to Return to Base mode")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

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

        # Widened region for smoother response
        self.regions = {
            'right': clamp(safe_min(ranges[60:120]), 0.0, 3.0),
            'front': clamp(safe_min(ranges[0:15] + ranges[-15:]), 0.0, 3.0),
            'left': clamp(safe_min(ranges[240:300]), 0.0, 3.0)
        }

    def change_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"State changed from {self.state} to {new_state}")
            self.state = new_state
            if new_state == 3:
                self.turning_start_time = time.time()  # Start timer for smart turn

    # ===================== ARUCO CALLBACKS ======================
    def aruco_id_callback(self, msg):
        self.last_marker_id = msg.data
        # If both ID and pose available, handle
        if self.last_marker_pose is not None:
            self._handle_aruco_detection()

    def aruco_pose_callback(self, msg):
        # Save latest pose
        q = msg.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.last_marker_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            yaw
        )
        # If both ID and pose available, handle
        if self.last_marker_id is not None:
            self._handle_aruco_detection()

    def _handle_aruco_detection(self):
        """Store marker pose on first sight; return if seen again."""
        marker_id = self.last_marker_id
        marker_pose = self.last_marker_pose

        if marker_id is None or marker_pose is None:
            return  # Defensive

        if marker_id not in self.seen_markers:
            # First time seeing this marker
            self.seen_markers[marker_id] = marker_pose
            self.get_logger().info(f"First time seeing marker {marker_id}, storing pose: ({marker_pose[0]:.2f}, {marker_pose[1]:.2f}, {marker_pose[2]:.2f})")
        else:
            # Already seen before! Go to that pose.
            saved_pose = self.seen_markers[marker_id]
            self.return_pose = saved_pose
            if not self.goto_active:
                self.get_logger().info(f"Seen marker {marker_id} again! Returning to its original pose.")
                self.state = 10
                self.goto_active = True
                self.update_status(f"Returning to pose of marker {marker_id}")
        # Reset to avoid repeated triggers
        self.last_marker_id = None
        self.last_marker_pose = None

    # ===================== WALL FOLLOWING FSM ======================
    def control_loop(self):
        if self.stopped:
            return

        if not self.active:
            self.update_status("Idle: Waiting for Start Marker")
            return

        # --- GO TO POSE MODE ---
        if self.state == 10:
            self.go_to_pose_logic()
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
        min_dist = self.min_dist
        front = self.regions['front']
        right = self.regions['right']
        left = self.regions['left']

        # --- State 0: Search for Wall ---
        if self.state == 0:
            if front < 0.8 or right < d - 0.05:
                self.change_state(3)  # Start smart turn
            elif right < d + 0.1:
                self.change_state(2)

        # --- State 2: Relaxed Wall Following ---
        elif self.state == 2:
            if front < 0.45 or right < min_dist:
                self.change_state(3)
                return

            if right > d + 0.5 and front > 0.5:
                msg.linear.x = 0.11
                msg.angular.z = 0.22  # Slight left
                self.update_status("Open edge, gently curving left.")
                self.cmd_pub.publish(msg)
                return

            error = right - d
            if abs(error) < 0.07:
                msg.linear.x = 0.14
                msg.angular.z = 0.0
                self.update_status("Following wall (deadband: straight)")
            else:
                msg.linear.x = 0.13
                msg.angular.z = -1.1 * error
                msg.angular.z = max(min(msg.angular.z, 0.4), -0.4)
                self.update_status(f"Following wall | Error: {error:.2f}")

            self.cmd_pub.publish(msg)
            return

        # --- State 3: Smart Turn (corner) ---
        elif self.state == 3:
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # gentle turn
            self.update_status("Turning left at corner (smart turn)")
            if right < d + 0.08 and right > d - 0.12 and front > min_dist:
                self.change_state(2)
            elif self.turning_start_time and (time.time() - self.turning_start_time > 5.0):
                self.get_logger().warn("Smart turn timeout—resetting to wall search")
                self.change_state(0)

        # Publish correct command for each state if not handled above:
        if self.state == 0:
            msg.linear.x = 0.10
            msg.angular.z = -0.20
            self.update_status("Searching for wall")
        elif self.state == 2:
            pass

        self.cmd_pub.publish(msg)

    # ===================== RETURN TO POSE LOGIC ======================
    def go_to_pose_logic(self):
        if self.current_position is None:
            self.update_status("Waiting for odometry for go-to-pose")
            return
        x, y = self.current_position
        yaw = self.current_yaw
        goal_x, goal_y, goal_theta = self.return_pose
        dx = goal_x - x
        dy = goal_y - y
        distance = sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = atan2(dy, dx)
        angle_diff = angle_to_goal - yaw
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        cmd = Twist()
        if distance > 0.08:
            if abs(angle_diff) > 0.15:
                cmd.angular.z = 0.3 * angle_diff
            else:
                cmd.linear.x = 0.14
                cmd.angular.z = 0.2 * angle_diff
            self.cmd_pub.publish(cmd)
            self.update_status(f"Returning: d={distance:.2f}, angle={angle_diff:.2f}")
        else:
            theta_error = goal_theta - yaw
            while theta_error > pi:
                theta_error -= 2 * pi
            while theta_error < -pi:
                theta_error += 2 * pi
            if abs(theta_error) > 0.1:
                cmd.angular.z = 0.3 * theta_error
                self.cmd_pub.publish(cmd)
                self.update_status(f"Aligning at goal: theta_error={theta_error:.2f}")
            else:
                self.cmd_pub.publish(Twist())  # Stop
                self.goto_active = False
                self.state = 0  # Switch back to wall following
                self.update_status("Arrived at goal pose!")

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
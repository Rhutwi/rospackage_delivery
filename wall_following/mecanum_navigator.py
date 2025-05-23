#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import random
import time

class MecanumSearchNavigator(Node):
    def __init__(self):
        super().__init__('mecanum_search_navigator')
        self.get_logger().info("Mecanum Search Navigator started")

        # Subscribers
        self.create_subscription(Odometry,    '/odometry/filtered',        self.odom_callback,    10)
        self.create_subscription(PoseStamped,'/marker_pose', self.marker_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control parameters
        self.kp_linear      = 0.5    # approach gains
        self.kp_angular     = 1.0
        self.max_lin        = 0.4
        self.max_lat        = 0.4
        self.max_ang        = 1.0
        self.goal_tolerance = 0.1
        self.yaw_tolerance  = 0.1

        # Roaming parameters
        self.roam_lin       = 0.2    # constant forward speed
        self.roam_ang       = 0.2    # constant rotate speed

        # State
        self.current_pose = None      # (x,y,yaw)
        self.marker_goal  = None      # (x,y,yaw)
        self.mode         = 'roam'    # 'roam' or 'approach'

        # Timer
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
        # quaternion → yaw
        yaw = math.atan2(
            2.0*(qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        )
        self.current_pose = (pos.x, pos.y, yaw)

    def marker_callback(self, msg: PoseStamped):
        # when a marker is detected, switch to approach mode
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        oq = msg.pose.orientation
        qx, qy, qz, qw = oq.x, oq.y, oq.z, oq.w
        gyaw = math.atan2(
            2.0*(qw*qz + qx*qy),
            1.0 - 2.0*(qy*qy + qz*qz)
        )
        self.marker_goal = (gx, gy, gyaw)
        self.mode = 'approach'
        self.get_logger().info(f"Marker detected – switching to approach: ({gx:.2f}, {gy:.2f})")

    def control_loop(self):
        if self.current_pose is None:
            return

        twist = Twist()

        if self.mode == 'roam':
            # simple spiral/roam: forward + slow rotation
            twist.linear.x  = self.roam_lin
            twist.angular.z = self.roam_ang

        elif self.mode == 'approach' and self.marker_goal:
            # compute approach errors
            cx, cy, cyaw = self.current_pose
            gx, gy, gyaw = self.marker_goal

            ex = gx - cx
            ey = gy - cy
            er_x =  math.cos(cyaw)*ex + math.sin(cyaw)*ey
            er_y = -math.sin(cyaw)*ex + math.cos(cyaw)*ey
            yaw_err = self._angle_diff(gyaw, cyaw)

            # if within tolerance, stop and revert to roaming
            if abs(er_x) < self.goal_tolerance and abs(er_y) < self.goal_tolerance:
                self.get_logger().info("Reached marker goal – resuming roam")
                self.mode = 'roam'
                self.marker_goal = None
                twist = Twist()  # stop
            else:
                # proportional holonomic approach
                vx = max(-self.max_lin,  min(self.max_lin,  self.kp_linear * er_x))
                vy = max(-self.max_lat,  min(self.max_lat,  self.kp_linear * er_y))
                wz = max(-self.max_ang,  min(self.max_ang,  self.kp_angular * yaw_err))
                twist.linear.x  = vx
                twist.linear.y  = vy
                twist.angular.z = wz

        self.cmd_pub.publish(twist)

    @staticmethod
    def _angle_diff(target, source):
        a = target - source
        return math.atan2(math.sin(a), math.cos(a))

def main(args=None):
    rclpy.init(args=args)
    node = MecanumSearchNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down search navigator")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

class MarkerTouchVerifier(Node):
    """Verifies pointer_tip proximity to marker for pickup/delivery."""

    def __init__(self):
        super().__init__('marker_touch_verifier')
        self.declare_parameter('marker_topic',   '/aruco/map_pose')
        self.declare_parameter('pointer_frame',  'pointer_tip')
        self.declare_parameter('target_frame',   'map')
        self.declare_parameter('tolerance_m',    0.05)

        self.marker_topic  = self.get_parameter('marker_topic').value
        self.pointer_frame = self.get_parameter('pointer_frame').value
        self.target_frame  = self.get_parameter('target_frame').value
        self.tolerance     = self.get_parameter('tolerance_m').value

        self.tf_buf  = Buffer()
        self.tf_list = TransformListener(self.tf_buf, self)

        self.pickup_pub   = self.create_publisher(Bool, '/pickup_verified',   10)
        self.delivery_pub = self.create_publisher(Bool, '/delivery_verified', 10)
        self.id_pub       = self.create_publisher(Int32, '/last_verified_marker_id', 10)
        self.last_id = None

        self.create_subscription(PoseStamped, self.marker_topic, self.cb_marker, 10)
        self.create_subscription(Int32,       '/aruco/id',       self.cb_id,     10)

        self.get_logger().info("MarkerTouchVerifier initialized")

    def cb_id(self, msg: Int32):
        self.last_id = msg.data

    def cb_marker(self, msg: PoseStamped):
        try:
            tf = self.tf_buf.lookup_transform(
                self.target_frame, self.pointer_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Create a pose stub at pointer_tip, then transform it
            stub = PoseStamped(header=msg.header)
            stub.header.frame_id = self.pointer_frame
            stub.header.stamp    = msg.header.stamp
            ptr = do_transform_pose(stub, tf)

            dx = msg.pose.position.x - ptr.pose.position.x
            dy = msg.pose.position.y - ptr.pose.position.y
            dz = msg.pose.position.z - ptr.pose.position.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            success = dist <= self.tolerance

            self.pickup_pub.publish(Bool(data=success))
            self.delivery_pub.publish(Bool(data=success))
            if success and self.last_id is not None:
                self.id_pub.publish(Int32(data=self.last_id))
                self.get_logger().info(f"Touch verified (ID={self.last_id}, d={dist:.3f}m)")
            elif not success:
                self.get_logger().info(f"Too far: d={dist:.3f}m")
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTouchVerifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

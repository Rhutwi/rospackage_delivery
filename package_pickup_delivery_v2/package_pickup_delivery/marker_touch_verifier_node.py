#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
import math


class MarkerTouchVerifier(Node):
    """Verifies if pointer_tip is within tolerance of a marker in map frame."""

    def __init__(self):
        super().__init__('marker_touch_verifier')

        # Declare and fetch parameters
        self.declare_parameter('marker_topic', '/aruco/map_pose')
        self.declare_parameter('pointer_frame', 'pointer_tip')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('tolerance_m', 0.05)

        self.marker_topic  = self.get_parameter('marker_topic').value
        self.pointer_frame = self.get_parameter('pointer_frame').value
        self.target_frame  = self.get_parameter('target_frame').value
        self.tolerance     = self.get_parameter('tolerance_m').value

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pickup_pub   = self.create_publisher(Bool, '/pickup_verified', 10)
        self.delivery_pub = self.create_publisher(Bool, '/delivery_verified', 10)
        self.id_pub       = self.create_publisher(Int32, '/last_verified_marker_id', 10)

        # Subscriptions
        self.last_id = None
        self.create_subscription(Int32, '/aruco/id', self.cb_id, 10)
        self.create_subscription(PoseStamped, self.marker_topic, self.cb_marker, 10)

        self.get_logger().info("🧪 MarkerTouchVerifier ready and listening...")

    def cb_id(self, msg: Int32):
        self.last_id = msg.data

    def cb_marker(self, marker_pose: PoseStamped):
        # Try to lookup pointer_tip in map frame
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.pointer_frame,
                marker_pose.header.stamp,
                timeout=Duration(seconds=1.0)
            )

            # Stub pose at pointer_tip frame
            stub_pose = PoseStamped()
            stub_pose.header.frame_id = self.pointer_frame
            stub_pose.header.stamp = marker_pose.header.stamp

            # Transform pointer tip to map frame
            pointer_pose = do_transform_pose(stub_pose, tf_stamped)

            # Compute Euclidean distance
            dx = marker_pose.pose.position.x - pointer_pose.pose.position.x
            dy = marker_pose.pose.position.y - pointer_pose.pose.position.y
            dz = marker_pose.pose.position.z - pointer_pose.pose.position.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            success = dist <= self.tolerance

            # Publish results
            self.pickup_pub.publish(Bool(data=success))
            self.delivery_pub.publish(Bool(data=success))

            if success and self.last_id is not None:
                self.id_pub.publish(Int32(data=self.last_id))
                self.get_logger().info(
                    f"✅ Touch verified for marker_{self.last_id} (distance = {dist:.4f} m)"
                )
            elif not success:
                self.get_logger().info(f"❌ Too far from marker: d = {dist:.4f} m")

        except TransformException as e:
            self.get_logger().warn(f"⚠ TF transform failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTouchVerifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

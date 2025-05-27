#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ExtrapolationException,
    TransformException,
)
import tf2_geometry_msgs  # registers the do_transform_pose helper


class TFPoseTransformer(Node):
    """
    Transforms incoming /aruco/pose (in 'camera_link') into a PoseStamped
    in the 'map' frame, publishing on /aruco/map_pose.  Also watches
    /aruco/id so it can log "marker_<ID>" → map.
    """

    def __init__(self):
        super().__init__('tf_pose_transformer')

        # --- parameters ---
        self.declare_parameter('input_pose_topic',  '/aruco/pose')
        self.declare_parameter('input_id_topic',    '/aruco/id')
        self.declare_parameter('output_pose_topic', '/aruco/map_pose')
        self.declare_parameter('target_frame',      'map')

        self.in_pose   = self.get_parameter('input_pose_topic').value
        self.in_id     = self.get_parameter('input_id_topic').value
        self.out_pose  = self.get_parameter('output_pose_topic').value
        self.tf_target = self.get_parameter('target_frame').value

        # --- tf2 setup ---
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- publisher + state ---
        self.map_pub     = self.create_publisher(PoseStamped, self.out_pose, 10)
        self.current_id  = None

        # --- subscriptions ---
        self.create_subscription(Int32,       self.in_id,   self._id_cb,   10)
        self.create_subscription(PoseStamped, self.in_pose, self._pose_cb, 10)

        self.get_logger().info(
            f"TFPoseTransformer:\n"
            f"  → listening  : {self.in_pose}\n"
            f"  → id topic   : {self.in_id}\n"
            f"  → publishing : {self.out_pose} (frame `{self.tf_target}`)"
        )

    def _id_cb(self, msg: Int32):
        # remember the latest marker ID for logging
        self.current_id = msg.data

    def _pose_cb(self, msg: PoseStamped):
        # try to lookup the camera_link→map transform at exactly msg.header.stamp
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.tf_target,        # target frame
                msg.header.frame_id,   # source frame (usually 'camera_link')
                Time())                # lookup the latest transform
        except LookupException as e:
            self.get_logger().warn(f"LookupException: {e}")
            return
        except ExtrapolationException as e:
            self.get_logger().warn(f"ExtrapolationException: {e}")
            return
        except TransformException as e:
            self.get_logger().warn(f"TransformException: {e}")
            return

        # actually do the PoseStamped → PoseStamped in map
        out = tf2_geometry_msgs.do_transform_pose(msg, tf_stamped)
        out.header.frame_id = self.tf_target

        # publish it
        self.map_pub.publish(out)

        # log a nicely human‐readable line
        if self.current_id is not None:
            self.get_logger().info(
                f"Transformed marker_{self.current_id} into `{self.tf_target}` frame"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TFPoseTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Int32
# from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, TransformException
# import tf2_geometry_msgs

# class TFPoseTransformer(Node):
#     """Transforms /aruco/pose from camera_link → map frame."""

#     def __init__(self):
#         super().__init__('tf_pose_transformer')

#         self.declare_parameter('input_pose_topic',  '/aruco/pose')
#         self.declare_parameter('input_id_topic',    '/aruco/id')
#         self.declare_parameter('output_pose_topic', '/aruco/map_pose')
#         self.declare_parameter('target_frame',      'map')

#         self.in_pose  = self.get_parameter('input_pose_topic').value
#         self.in_id    = self.get_parameter('input_id_topic').value
#         self.out_pose = self.get_parameter('output_pose_topic').value
#         self.tf_target= self.get_parameter('target_frame').value

#         self.tf_buf    = Buffer()
#         self.tf_list  = TransformListener(self.tf_buf, self)
#         self.map_pub  = self.create_publisher(PoseStamped, self.out_pose, 10)
#         self.current_id = None

#         self.create_subscription(Int32,      self.in_id,   self.id_cb,   10)
#         self.create_subscription(PoseStamped,self.in_pose, self.pose_cb, 10)

#         self.get_logger().info("TFPoseTransformer initialized")

#     def id_cb(self, msg: Int32):
#         self.current_id = msg.data

#     def pose_cb(self, msg: PoseStamped):
#         try:
#             tf = self.tf_buf.lookup_transform(
#                 self.tf_target, msg.header.frame_id,
#                 msg.header.stamp,
#                 timeout=rclpy.duration.Duration(seconds=1.0)
#             )
#             out = tf2_geometry_msgs.do_transform_pose(msg, tf)
#             out.header.frame_id = self.tf_target
#             self.map_pub.publish(out)

#             if self.current_id is not None:
#                 self.get_logger().info(f"Transformed marker_{self.current_id} → {self.tf_target}")
#         except LookupException as e:
#             self.get_logger().warn(f"LookupException: {e}")
#         except ExtrapolationException as e:
#             self.get_logger().warn(f"ExtrapolationException: {e}")
#         except TransformException as e:
#             self.get_logger().warn(f"TransformException: {e}")
#         except Exception as e:
#             self.get_logger().error(f"Unexpected TF error: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TFPoseTransformer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

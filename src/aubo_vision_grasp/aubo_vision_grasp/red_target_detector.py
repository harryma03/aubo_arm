#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


class RedTargetDetector(Node):

    def __init__(self):
        super().__init__('red_target_detector')

        self.bridge = CvBridge()

        self.depth = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.depth_frame = 'camera_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10
        )

        self.count_pub = self.create_publisher(
            Int32,
            '/vision/target_count',
            10
        )

        self.mask_pub = self.create_publisher(
            Image,
            '/vision/red_mask',
            10
        )

        self.create_subscription(
            Image,
            '/camera/workcell_camera/depth/image_raw',
            self.depth_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            CameraInfo,
            '/camera/workcell_camera/depth/camera_info',
            self.info_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            '/camera/workcell_camera/image_raw',
            self.rgb_cb,
            qos_profile_sensor_data
        )

        self.last_log_time = 0.0

        self.get_logger().info('Red target detector started')

    def info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        if msg.header.frame_id:
            self.depth_frame = msg.header.frame_id

    def depth_cb(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            ).astype(np.float32)

            if msg.header.frame_id:
                self.depth_frame = msg.header.frame_id

        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    @staticmethod
    def quat_to_rot(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def rgb_cb(self, msg):

        if self.depth is None:
            return

        if self.fx is None:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 红色有两个 HSV 区间
        mask1 = cv2.inRange(
            hsv,
            np.array([0, 100, 80]),
            np.array([10, 255, 255])
        )

        mask2 = cv2.inRange(
            hsv,
            np.array([170, 100, 80]),
            np.array([179, 255, 255])
        )

        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            count_msg = Int32()
            count_msg.data = 0
            self.count_pub.publish(count_msg)
            return

        # -----------------------------------------------------
        # Camera -> base_link transform.
        # All contours in this frame use the same TF.
        # -----------------------------------------------------
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                self.depth_frame,
                Time()
            )
        except Exception:
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        R = self.quat_to_rot(
            q.x,
            q.y,
            q.z,
            q.w
        )

        translation = np.array([
            t.x,
            t.y,
            t.z
        ])

        # -----------------------------------------------------
        # Multi-object detection:
        # evaluate every red contour independently.
        #
        # Only objects on the SOURCE TABLE are accepted.
        # Therefore red cubes already placed in the receiving
        # bin (x ~= -0.605) will be rejected automatically.
        # -----------------------------------------------------
        valid_targets = []

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 100:
                continue

            M = cv2.moments(contour)

            if M['m00'] == 0:
                continue

            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])

            # Use only depth pixels belonging to this contour.
            # Median depth provides robustness against noise.
            object_mask = np.zeros(
                mask.shape,
                dtype=np.uint8
            )

            cv2.drawContours(
                object_mask,
                [contour],
                -1,
                255,
                -1
            )

            valid_depth = (
                (object_mask > 0) &
                np.isfinite(self.depth) &
                (self.depth > 0.05) &
                (self.depth < 3.0)
            )

            depths = self.depth[valid_depth]

            if depths.size < 10:
                continue

            Z = float(np.median(depths))

            # Camera optical coordinates.
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            p_cam = np.array([
                X,
                Y,
                Z
            ])

            p_base = R @ p_cam + translation

            bx = float(p_base[0])
            by = float(p_base[1])
            bz = float(p_base[2])

            # -------------------------------------------------
            # Current source-table ROI in base_link.
            #
            # Table objects:
            #   x roughly [-0.30, +0.30]
            #   y roughly [-1.10, -0.47]
            #
            # Receiving bin:
            #   x roughly -0.605
            #
            # Therefore objects already inside the bin are
            # excluded here.
            # -------------------------------------------------
            if not (
                -0.30 < bx < 0.30 and
                -1.10 < by < -0.47 and
                -0.02 < bz < 0.10
            ):
                continue

            valid_targets.append({
                'area': area,
                'u': u,
                'v': v,
                'depth': Z,
                'p_base': p_base,
            })

        # Always publish the current segmentation mask.
        debug = self.bridge.cv2_to_imgmsg(
            mask,
            encoding='mono8'
        )
        debug.header = msg.header
        self.mask_pub.publish(debug)

        count_msg = Int32()
        count_msg.data = len(valid_targets)
        self.count_pub.publish(count_msg)

        if not valid_targets:
            return

        # Choose one valid object.
        # For equal-size cubes this is deterministic enough;
        # after it is removed, the next callback selects one
        # of the remaining objects.
        target = max(
            valid_targets,
            key=lambda item: item['area']
        )

        u = target['u']
        v = target['v']
        Z = target['depth']
        p_base = target['p_base']

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'

        pose.pose.position.x = float(p_base[0])
        pose.pose.position.y = float(p_base[1])
        pose.pose.position.z = float(p_base[2])

        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time > 1.0:
            self.last_log_time = now

            self.get_logger().info(
                f'red targets on table={len(valid_targets)} | '
                f'selected pixel=({u},{v}) '
                f'depth={Z:.3f} m | '
                f'base=({p_base[0]:.3f}, '
                f'{p_base[1]:.3f}, '
                f'{p_base[2]:.3f})'
            )



def main(args=None):
    rclpy.init(args=args)
    node = RedTargetDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

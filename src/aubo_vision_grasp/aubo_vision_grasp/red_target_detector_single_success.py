#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
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
            return

        contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(contour) < 100:
            return

        M = cv2.moments(contour)

        if M['m00'] == 0:
            return

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        # 只取红色目标区域内的有效深度，使用中值抗噪声
        object_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(
            object_mask,
            [contour],
            -1,
            255,
            -1
        )

        valid = (
            (object_mask > 0) &
            np.isfinite(self.depth) &
            (self.depth > 0.05) &
            (self.depth < 3.0)
        )

        depths = self.depth[valid]

        if depths.size < 10:
            return

        Z = float(np.median(depths))

        # camera optical coordinates
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        p_cam = np.array([X, Y, Z])

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

        p_base = R @ p_cam + np.array([
            t.x,
            t.y,
            t.z
        ])

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'

        pose.pose.position.x = float(p_base[0])
        pose.pose.position.y = float(p_base[1])
        pose.pose.position.z = float(p_base[2])

        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        debug = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        debug.header = msg.header
        self.mask_pub.publish(debug)

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time > 1.0:
            self.last_log_time = now

            self.get_logger().info(
                f'red target pixel=({u},{v}) '
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

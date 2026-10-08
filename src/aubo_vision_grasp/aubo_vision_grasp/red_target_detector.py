#!/usr/bin/env python3

import cv2
import math
import numpy as np
import os
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

try:
    from .system_config import load_system_config
except ImportError:
    from system_config import load_system_config


class RedTargetDetector(Node):

    # Capacity limit, not an expected target count.  The detector still
    # publishes however many confident objects are visible in each frame.
    MAX_SPLIT_PARTS = 8

    def __init__(self):
        super().__init__('red_target_detector')

        config = load_system_config()
        workcell = config.workcell
        vision = config.vision
        self.workcell_x = workcell.x
        self.workcell_y = workcell.y
        self.workcell_z = workcell.z
        self.workcell_dz = workcell.dz
        self.hopper_x = workcell.hopper_x
        self.hopper_y = workcell.hopper_y
        self.hopper_top_z = workcell.hopper_top_z
        self.hopper_dz = workcell.hopper_dz
        self.target_split_enable = vision.split_enable
        self.target_split_peak_ratio = vision.split_peak_ratio
        self.target_split_min_area_ratio = vision.split_min_area_ratio
        self.target_split_count_tolerance = vision.split_count_tolerance
        self.split_candidate_min_ratio = vision.split_candidate_min_ratio
        self.split_candidate_max_ratio = vision.split_candidate_max_ratio
        self.cube_size_x = workcell.cube_x
        self.cube_size_y = workcell.cube_y
        self.min_target_z = vision.min_target_z + self.hopper_dz
        self.max_target_z = vision.max_target_z + self.hopper_dz
        self.max_depth_spread = vision.max_depth_spread
        self.min_area_ratio = vision.min_area_ratio
        self.max_area_ratio = vision.max_area_ratio
        self.display_enabled = (
            vision.display_enabled
            and bool(os.environ.get('DISPLAY'))
        )
        self.display_window = 'AUBO RGB-D Vision Debug'
        self.display_error_logged = False
        self.rgbd_sync_slop = vision.sync_slop
        self.processing_rate = max(vision.processing_rate, 0.5)
        self.rgb_topic = vision.rgb_topic
        self.depth_topic = vision.depth_topic
        self.camera_info_topic = vision.camera_info_topic

        self.bridge = CvBridge()

        self.depth = None
        self.latest_depth_msg = None
        self.latest_rgb_msg = None
        self.depth_stamp_ns = None
        self.rgb_sequence = 0
        self.last_processed_rgb_sequence = -1
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.depth_frame = 'camera_link'
        self.rgb_messages = 0
        self.depth_messages = 0
        self.camera_info_messages = 0
        self.synced_frames = 0
        self.published_frames = 0
        self.diagnostic_frames = 0
        self.process_timer_ticks = 0
        self.status_timer_ticks = 0
        self.tf_failures = 0
        self.last_stamp_delta = None
        self.last_process_duration_ms = None
        self.previous_published_positions = None
        self.last_stage = 'INITIALIZING'
        self.last_rgb_wall_time = None
        self.last_depth_wall_time = None
        self.last_pipeline_update = time.monotonic()
        self.last_publish_update = None

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

        # New scene-level interface.  The legacy target_pose and target_count
        # topics remain published for compatibility with existing tools.
        self.target_array_pub = self.create_publisher(
            PoseArray,
            '/vision/target_array',
            10
        )

        # Number of red connected components whose depth centroid lies in the
        # source hopper.  This remains non-zero when shape filters reject a
        # touching/merged contour, allowing the queue to distinguish
        # "source really empty" from "red source objects were rejected".
        self.source_evidence_pub = self.create_publisher(
            Int32,
            '/vision/source_evidence_count',
            10,
        )

        # Raw merged regions that cannot yet be separated with confidence.
        # The queue mirrors these as conservative MoveIt obstacles instead of
        # inventing a grasp point at the merged contour centroid.
        self.ambiguous_region_pub = self.create_publisher(
            PoseArray,
            '/vision/ambiguous_regions',
            10,
        )

        self.mask_pub = self.create_publisher(
            Image,
            '/vision/red_mask',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            10
        )

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.info_cb,
            qos_profile_sensor_data
        )

        # Subscriptions only cache the newest images. A bounded-rate steady
        # timer performs OpenCV, depth and TF work so camera callbacks cannot
        # starve each other and the queue's wall-clock timeout stays aligned.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_cb,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / self.processing_rate,
            self.process_latest_frame,
            clock=self.steady_clock,
        )
        self.create_timer(
            2.0,
            self.pipeline_status_cb,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'Red target detector started | '
            f'rgb={self.rgb_topic} | depth={self.depth_topic} | '
            f'processing={self.processing_rate:.1f}Hz | '
            f'sync_gate={self.rgbd_sync_slop:.3f}s | '
            f'display={self.display_enabled} | '
            f'debug_topic=/vision/debug_image'
        )

    def info_cb(self, msg):
        self.camera_info_messages += 1
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        if msg.header.frame_id:
            self.depth_frame = msg.header.frame_id

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def depth_cb(self, depth_msg):
        self.depth_messages += 1
        self.latest_depth_msg = depth_msg
        self.last_depth_wall_time = time.monotonic()

    def rgb_cb(self, rgb_msg):
        self.rgb_messages += 1
        self.rgb_sequence += 1
        self.latest_rgb_msg = rgb_msg
        self.last_rgb_wall_time = time.monotonic()

    def process_latest_frame(self):
        """Process the newest synchronized pair at a predictable rate."""
        self.process_timer_ticks += 1
        self.last_stage = 'PROCESS_TIMER'
        rgb_msg = self.latest_rgb_msg
        depth_msg = self.latest_depth_msg
        if rgb_msg is None or depth_msg is None or self.fx is None:
            self.last_stage = 'WAITING_FOR_CAMERA_INPUT'
            return

        if self.rgb_sequence == self.last_processed_rgb_sequence:
            return

        rgb_stamp_ns = self.stamp_to_ns(rgb_msg.header.stamp)
        depth_stamp_ns = self.stamp_to_ns(depth_msg.header.stamp)
        self.last_stamp_delta = abs(rgb_stamp_ns - depth_stamp_ns) / 1e9
        if self.last_stamp_delta > self.rgbd_sync_slop:
            self.last_stage = 'RGBD_SYNC_REJECTED'
            self.last_processed_rgb_sequence = self.rgb_sequence
            self._record_frame(
                raw_contours=0,
                evaluated_contours=0,
                positions=[],
                published=False,
                reason='RGBD_SKEW',
                process_start=time.monotonic(),
            )
            return

        process_start = time.monotonic()
        try:
            self.depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            ).astype(np.float32)
        except Exception as error:
            self.last_stage = 'DEPTH_CONVERSION_FAILED'
            self.get_logger().error(f'Depth conversion failed: {error}')
            self.last_processed_rgb_sequence = self.rgb_sequence
            self._record_frame(
                raw_contours=0,
                evaluated_contours=0,
                positions=[],
                published=False,
                reason='DEPTH_CONVERSION_FAILED',
                process_start=process_start,
            )
            return

        if depth_msg.header.frame_id:
            self.depth_frame = depth_msg.header.frame_id
        self.depth_stamp_ns = depth_stamp_ns
        self.last_processed_rgb_sequence = self.rgb_sequence
        self.synced_frames += 1
        self.last_pipeline_update = time.monotonic()
        self.last_stage = 'DEPTH_CONVERTED'
        self.last_stage = 'RGB_PROCESSING'
        try:
            outcome = self.process_rgb_frame(rgb_msg)
        except Exception as error:
            self.last_stage = 'RGB_PROCESS_EXCEPTION'
            self.get_logger().error(
                'RGB-D processing exception at stage '
                f'{self.last_stage}: {error}\n{traceback.format_exc()}'
            )
            outcome = {
                'raw_contours': 0,
                'evaluated_contours': 0,
                'positions': [],
                'published': False,
                'reason': self.last_stage,
            }
        self.last_stage = outcome['reason']
        self._record_frame(process_start=process_start, **outcome)

    @staticmethod
    def _format_positions(positions):
        if not positions:
            return '[]'
        return '[' + ', '.join(
            f'({x:.3f},{y:.3f},{z:.3f})'
            for x, y, z in positions
        ) + ']'

    def _publish_debug_frame(
        self,
        image_msg,
        frame,
        status,
        raw_contours=None,
        valid_targets=None,
    ):
        """Publish and optionally display an annotated detector frame."""
        annotated = frame.copy()
        raw_contours = raw_contours or []
        valid_targets = valid_targets or []

        for contour in raw_contours:
            x, y, width, height = cv2.boundingRect(contour)
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                (0, 255, 255),
                2,
            )

        for index, target in enumerate(valid_targets, start=1):
            x, y, width, height = target['bbox']
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2,
            )
            bx, by, bz = target['p_base']
            label = (
                f'#{index} px=({target["u"]},{target["v"]}) '
                f'base=({bx:.3f},{by:.3f},{bz:.3f})'
            )
            cv2.putText(
                annotated,
                label,
                (x, max(y - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            f'{status} | raw={len(raw_contours)} '
            f'targets={len(valid_targets)}',
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if valid_targets else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding='bgr8',
            )
            debug_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as error:
            self.get_logger().error(
                f'Debug image publication failed: {error}'
            )

        if not self.display_enabled:
            return

        try:
            cv2.imshow(self.display_window, annotated)
            cv2.waitKey(1)
        except cv2.error as error:
            self.display_enabled = False
            if not self.display_error_logged:
                self.display_error_logged = True
                self.get_logger().warn(
                    'OpenCV display disabled; use /vision/debug_image '
                    f'in RViz/rqt instead: {error}'
                )

    def close_debug_display(self):
        if not self.display_enabled:
            return
        try:
            cv2.destroyWindow(self.display_window)
            cv2.waitKey(1)
        except cv2.error:
            pass

    def _record_frame(
        self,
        raw_contours,
        evaluated_contours,
        positions,
        published,
        reason,
        process_start,
        rejections=None,
        source_evidence=None,
    ):
        """Log one compact, comparable diagnostic record per RGB frame."""
        positions = [tuple(float(value) for value in point)
                     for point in positions]
        self.diagnostic_frames += 1
        self.last_process_duration_ms = (
            time.monotonic() - process_start
        ) * 1000.0

        if self.previous_published_positions is None:
            delta = 'first'
        elif len(positions) != len(self.previous_published_positions):
            delta = 'count_changed'
        elif positions:
            max_delta = max(
                math.dist(previous, current)
                for previous, current in zip(
                    self.previous_published_positions,
                    positions,
                )
            )
            delta = f'{max_delta:.3f}'
        else:
            delta = '0.000'

        if published:
            self.previous_published_positions = positions

        rejection_text = ''
        if rejections:
            rejection_text = ' rejects=' + ','.join(
                f'{name}:{count}'
                for name, count in rejections.items()
                if count
            )
        evidence_text = (
            ''
            if source_evidence is None
            else f' source_evidence={int(source_evidence)}'
        )

        self.get_logger().info(
            f'[VISION FRAME {self.diagnostic_frames}] '
            f'rgb={self.rgb_sequence} depth={self.depth_messages} '
            f'sync={self.last_stamp_delta:.3f}s '
            f'raw={raw_contours} evaluated={evaluated_contours} '
            f'targets={len(positions)} delta={delta}m '
            f'published={published} reason={reason}{evidence_text}'
            f'{rejection_text} '
            f'process={self.last_process_duration_ms:.1f}ms '
            f'positions={self._format_positions(positions)}'
        )

    def pipeline_status_cb(self):
        self.status_timer_ticks += 1
        now = time.monotonic()
        if (
            self.last_publish_update is not None
            and now - self.last_publish_update <= 3.0
        ):
            return
        info_ready = self.fx is not None
        delta = (
            'unknown'
            if self.last_stamp_delta is None
            else f'{self.last_stamp_delta:.4f}s'
        )
        self.get_logger().warn(
            'RGB-D pipeline is not publishing a current target frame: '
            f'rgb={self.rgb_messages}, depth={self.depth_messages}, '
            f'camera_info={info_ready}({self.camera_info_messages}), '
            f'rgb_age={self._age_text(self.last_rgb_wall_time, now)}, '
            f'depth_age={self._age_text(self.last_depth_wall_time, now)}, '
            f'last_stage={self.last_stage}, '
            f'timer_ticks={self.process_timer_ticks}, '
            f'status_ticks={self.status_timer_ticks}, '
            f'last_stamp_delta={delta}, '
            f'allowed={self.rgbd_sync_slop:.4f}s, '
            f'accepted_frames={self.synced_frames}, '
            f'published_frames={self.published_frames}, '
            f'diagnostic_frames={self.diagnostic_frames}, '
            f'last_process_ms={self.last_process_duration_ms}, '
            f'tf_failures={self.tf_failures}'
        )

    @staticmethod
    def _age_text(last_time, now):
        if last_time is None:
            return 'never'
        return f'{now - last_time:.2f}s'

    @staticmethod
    def quat_to_rot(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def publish_target_array(self, image_msg, targets):
        """Publish every valid target from one RGB-D frame."""
        array_msg = PoseArray()
        array_msg.header.stamp = image_msg.header.stamp
        array_msg.header.frame_id = 'base_link'

        for target in targets:
            pose = PoseStamped().pose
            p_base = target['p_base']
            pose.position.x = float(p_base[0])
            pose.position.y = float(p_base[1])
            pose.position.z = float(p_base[2])
            pose.orientation.w = 1.0
            array_msg.poses.append(pose)

        self.target_array_pub.publish(array_msg)
        self.published_frames += 1
        self.last_publish_update = time.monotonic()

    def publish_ambiguous_regions(self, image_msg, measurements):
        array_msg = PoseArray()
        array_msg.header.stamp = image_msg.header.stamp
        array_msg.header.frame_id = 'base_link'
        for measurement in measurements:
            pose = Pose()
            point = measurement['p_base']
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            array_msg.poses.append(pose)
        self.ambiguous_region_pub.publish(array_msg)

    def _split_by_kmeans(self, component, x0, y0, part_count):
        pixels = np.column_stack(np.where(component > 0)).astype(np.float32)
        if len(pixels) < part_count * 20:
            return []

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            20,
            0.5,
        )
        _compactness, labels, _centers = cv2.kmeans(
            pixels,
            int(part_count),
            None,
            criteria,
            3,
            cv2.KMEANS_PP_CENTERS,
        )

        split_contours = []
        for label in range(int(part_count)):
            region = np.zeros_like(component)
            selected = pixels[labels.ravel() == label].astype(np.int32)
            if len(selected) == 0:
                continue
            region[selected[:, 0], selected[:, 1]] = 255
            region_contours, _ = cv2.findContours(
                region,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
                offset=(x0, y0),
            )
            split_contours.extend(
                item for item in region_contours
                if cv2.contourArea(item) >= 25
            )

        return split_contours

    def split_connected_contour(
        self,
        contour,
        mask,
        expected_single_area=None,
    ):
        """
        Try to split touching same-colour objects with distance markers.

        A normal single rectangular object usually produces one distance peak
        and is returned unchanged.  A connected pair can produce two peaks,
        in which case watershed returns separate image regions.  If the
        markers are ambiguous, the original contour is retained instead of
        inventing additional objects.
        """
        if not self.target_split_enable:
            return [contour]

        contour_area = cv2.contourArea(contour)
        if (
            expected_single_area is not None
            and contour_area
            < expected_single_area * self.target_split_min_area_ratio
        ):
            return [contour]

        x, y, width, height = cv2.boundingRect(contour)
        pad = 2
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(mask.shape[1], x + width + pad)
        y1 = min(mask.shape[0], y + height + pad)

        component = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, :, 0] -= x0
        shifted[:, :, 1] -= y0
        cv2.drawContours(component, [shifted], -1, 255, -1)

        distance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
        maximum = float(distance.max())
        if maximum <= 1.0:
            return [contour]

        peak_ratio = min(max(self.target_split_peak_ratio, 0.25), 0.80)
        peak_mask = (distance >= maximum * peak_ratio).astype(np.uint8)
        marker_count, markers = cv2.connectedComponents(peak_mask)

        # One peak means this is most likely one object.  If the contour is
        # nevertheless much larger than one projected cube, use a conservative
        # pixel clustering fallback for touching rectangles.
        if marker_count <= 2:
            if expected_single_area is None:
                return [contour]
            part_count = int(round(contour_area / expected_single_area))
            part_count = min(max(part_count, 2), self.MAX_SPLIT_PARTS)
            clustered = self._split_by_kmeans(
                component,
                x0,
                y0,
                part_count,
            )
            return clustered if len(clustered) >= 2 else [contour]

        sure_background = cv2.dilate(component, np.ones((3, 3), np.uint8))
        unknown = cv2.subtract(sure_background, peak_mask * 255)
        markers = markers + 1
        markers[unknown == 255] = 0

        watershed_image = cv2.cvtColor(component, cv2.COLOR_GRAY2BGR)
        cv2.watershed(watershed_image, markers)

        split_contours = []
        for label in range(2, marker_count + 1):
            region = np.zeros_like(component)
            region[markers == label] = 255
            region_contours, _ = cv2.findContours(
                region,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
                offset=(x0, y0),
            )
            split_contours.extend(
                item for item in region_contours
                if cv2.contourArea(item) >= 25
            )

        if len(split_contours) < 2:
            if expected_single_area is None:
                return [contour]
            part_count = int(round(contour_area / expected_single_area))
            part_count = min(max(part_count, 2), self.MAX_SPLIT_PARTS)
            clustered = self._split_by_kmeans(
                component,
                x0,
                y0,
                part_count,
            )
            return clustered if len(clustered) >= 2 else [contour]

        return split_contours

    def estimate_single_cube_area(self, contour, mask):
        """Estimate one cube's projected area at this contour depth."""
        if self.fx is None or self.fy is None or self.depth is None:
            return None

        object_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(object_mask, [contour], -1, 255, -1)
        valid_depth = (
            (object_mask > 0)
            & np.isfinite(self.depth)
            & (self.depth > 0.05)
            & (self.depth < 3.0)
        )
        depths = self.depth[valid_depth]
        if depths.size < 10:
            return None

        depth = float(np.median(depths))
        if depth <= 0.0:
            return None

        projected = abs(
            self.fx
            * self.fy
            * self.cube_size_x
            * self.cube_size_y
            / (depth * depth)
        )
        return max(projected, 100.0)

    def measure_contour_3d(self, contour, mask, rotation, translation):
        """Return a depth centroid in base_link before shape filtering."""
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None, 'MOMENT'

        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        object_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(object_mask, [contour], -1, 255, -1)
        valid_depth = (
            (object_mask > 0)
            & np.isfinite(self.depth)
            & (self.depth > 0.05)
            & (self.depth < 3.0)
        )
        depths = self.depth[valid_depth]
        if depths.size < 10:
            return None, 'DEPTH_PIXELS'

        depth_low, depth_high = np.percentile(depths, [20, 80])
        depth = float(np.median(depths))
        camera_point = np.array([
            (u - self.cx) * depth / self.fx,
            (v - self.cy) * depth / self.fy,
            depth,
        ])
        base_point = rotation @ camera_point + translation
        return {
            'u': u,
            'v': v,
            'depth': depth,
            'depth_spread': float(depth_high - depth_low),
            'p_base': base_point,
        }, None

    def is_source_hopper_point(self, point):
        bx, by, bz = (float(value) for value in point)
        return (
            self.hopper_x - 0.36 < bx < self.hopper_x + 0.36
            and self.hopper_y - 0.53 < by < self.hopper_y + 0.53
            and self.min_target_z < bz < self.max_target_z
        )

    def publish_source_evidence(self, count):
        message = Int32()
        message.data = int(count)
        self.source_evidence_pub.publish(message)

    def process_rgb_frame(self, msg):
        self.last_stage = 'RGB_CONVERSION'
        try:
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception:
            self.get_logger().warn('RGB image conversion failed')
            return {
                'raw_contours': 0,
                'evaluated_contours': 0,
                'positions': [],
                'published': False,
                'reason': 'RGB_CONVERSION_FAILED',
            }

        self.last_stage = 'RGB_SEGMENTATION'
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        debug_frame = bgr.copy()

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

        # A 5x5 close operation could artificially bridge cubes separated by
        # only a few pixels.  A 3x3 kernel still removes isolated noise while
        # leaving genuinely separate objects separate whenever possible.
        kernel = np.ones((3, 3), np.uint8)
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
            self.publish_source_evidence(0)
            self.publish_ambiguous_regions(msg, [])
            count_msg = Int32()
            count_msg.data = 0
            self.count_pub.publish(count_msg)
            self.publish_target_array(msg, [])
            self._publish_debug_frame(
                msg,
                debug_frame,
                'NO_RED_CONTOURS',
            )
            return {
                'raw_contours': 0,
                'evaluated_contours': 0,
                'positions': [],
                'published': True,
                'reason': 'NO_RED_CONTOURS',
                'source_evidence': 0,
            }

        # -----------------------------------------------------
        # Camera -> base_link transform.
        # All contours in this frame use the same TF.
        # -----------------------------------------------------
        self.last_stage = 'TF_LOOKUP'
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                self.depth_frame,
                Time()
            )
        except Exception:
            self.tf_failures += 1
            self._publish_debug_frame(
                msg,
                debug_frame,
                'TF_UNAVAILABLE',
                raw_contours=contours,
            )
            return {
                'raw_contours': len(contours),
                'evaluated_contours': 0,
                'positions': [],
                'published': False,
                'reason': 'TF_UNAVAILABLE',
            }

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
        self.last_stage = 'TARGET_FILTERING'
        valid_targets = []
        rejections = {
            'AREA_SMALL': 0,
            'AREA_RATIO': 0,
            'ASPECT': 0,
            'MOMENT': 0,
            'DEPTH_PIXELS': 0,
            'DEPTH_SPREAD': 0,
            'OUTSIDE_SOURCE': 0,
            'SPLIT_AMBIGUOUS': 0,
        }

        source_evidence_count = 0
        raw_measurements = []
        for raw_contour in contours:
            measurement, _failure = self.measure_contour_3d(
                raw_contour,
                mask,
                R,
                translation,
            )
            if (
                measurement is not None
                and self.is_source_hopper_point(measurement['p_base'])
            ):
                source_evidence_count += 1
                raw_measurements.append((raw_contour, measurement))

        # Publish before target_array so an empty target message is always
        # accompanied by evidence explaining whether the source is truly empty.
        self.publish_source_evidence(source_evidence_count)

        split_contour_count = 0
        ambiguous_measurements = []

        for raw_contour, raw_measurement in raw_measurements:

            expected_single_area = self.estimate_single_cube_area(
                raw_contour,
                mask,
            )
            candidate_contours = self.split_connected_contour(
                raw_contour,
                mask,
                expected_single_area,
            )
            split_contour_count += len(candidate_contours)

            # A large connected contour is never allowed to become one
            # synthetic target at its centroid. Infer how many physical cubes
            # its projected area represents and require the splitter to return
            # exactly that many plausible pieces. ``ceil`` with a tolerance is
            # intentionally safer than ``round`` near N+0.5, where the old
            # code alternated between three and four scene targets.
            if expected_single_area is None:
                ambiguous_measurements.append(raw_measurement)
                rejections['SPLIT_AMBIGUOUS'] += 1
                continue

            raw_area = cv2.contourArea(raw_contour)
            raw_area_ratio = raw_area / expected_single_area
            if raw_area_ratio >= self.target_split_min_area_ratio:
                expected_parts = int(math.ceil(
                    raw_area_ratio - self.target_split_count_tolerance
                ))
                expected_parts = min(
                    max(expected_parts, 2),
                    self.MAX_SPLIT_PARTS,
                )
                candidate_ratios = [
                    cv2.contourArea(item) / expected_single_area
                    for item in candidate_contours
                ]
                coverage = (
                    sum(cv2.contourArea(item) for item in candidate_contours)
                    / max(raw_area, 1.0)
                )
                split_is_confident = (
                    len(candidate_contours) == expected_parts
                    and coverage >= 0.60
                    and all(
                        self.split_candidate_min_ratio <= ratio
                        <= self.split_candidate_max_ratio
                        for ratio in candidate_ratios
                    )
                )
                if not split_is_confident:
                    ambiguous_measurements.append(raw_measurement)
                    rejections['SPLIT_AMBIGUOUS'] += 1
                    continue

            for contour in candidate_contours:

                area = cv2.contourArea(contour)

                if area < 100:
                    rejections['AREA_SMALL'] += 1
                    continue

                if expected_single_area is not None:
                    area_ratio = area / expected_single_area
                    if not (
                        self.min_area_ratio
                        <= area_ratio
                        <= self.max_area_ratio
                    ):
                        rejections['AREA_RATIO'] += 1
                        continue

                _x, _y, width, height = cv2.boundingRect(contour)
                if width <= 0 or height <= 0:
                    rejections['ASPECT'] += 1
                    continue
                aspect_ratio = float(width) / float(height)
                if not 0.25 <= aspect_ratio <= 4.0:
                    rejections['ASPECT'] += 1
                    continue

                measurement, measurement_failure = self.measure_contour_3d(
                    contour,
                    mask,
                    R,
                    translation,
                )
                if measurement is None:
                    rejections[measurement_failure] += 1
                    continue

                if measurement['depth_spread'] > self.max_depth_spread:
                    rejections['DEPTH_SPREAD'] += 1
                    continue

                u = measurement['u']
                v = measurement['v']
                Z = measurement['depth']
                p_base = measurement['p_base']

            # -------------------------------------------------
            # Current source-hopper ROI in base_link.
            #
            # Table objects:
            #   x/y are relative to the current hopper center.
            #
            # Receiving bin:
            #   x roughly -0.605
            #
            # Therefore objects already inside the bin are
            # excluded here.
            # -------------------------------------------------
                if not self.is_source_hopper_point(p_base):
                    rejections['OUTSIDE_SOURCE'] += 1
                    continue

                valid_targets.append({
                    'area': area,
                    'u': u,
                    'v': v,
                    'bbox': (_x, _y, width, height),
                    'depth': Z,
                    'p_base': p_base,
                })

        # Always publish the current segmentation mask.
        self.last_stage = 'TARGET_PUBLISH'
        self.publish_ambiguous_regions(msg, ambiguous_measurements)
        self._publish_debug_frame(
            msg,
            debug_frame,
            'TARGETS_READY' if valid_targets else 'NO_VALID_TARGETS',
            raw_contours=contours,
            valid_targets=valid_targets,
        )
        debug = self.bridge.cv2_to_imgmsg(
            mask,
            encoding='mono8'
        )
        debug.header = msg.header
        self.mask_pub.publish(debug)

        valid_targets.sort(
            key=lambda item: (
                float(item['p_base'][0]),
                float(item['p_base'][1]),
                float(item['p_base'][2]),
            )
        )
        self.publish_target_array(msg, valid_targets)

        count_msg = Int32()
        count_msg.data = len(valid_targets)
        self.count_pub.publish(count_msg)

        if not valid_targets:
            return {
                'raw_contours': len(contours),
                'evaluated_contours': split_contour_count,
                'positions': [],
                'published': True,
                'reason': 'NO_VALID_TARGETS',
                'rejections': rejections,
                'source_evidence': source_evidence_count,
            }

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

        return {
            'raw_contours': len(contours),
            'evaluated_contours': split_contour_count,
            'positions': [
                tuple(float(value) for value in item['p_base'])
                for item in valid_targets
            ],
            'published': True,
            'reason': 'OK',
            'rejections': rejections,
            'source_evidence': source_evidence_count,
        }


def main(args=None):
    rclpy.init(args=args)
    node = RedTargetDetector()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM can shut down the rclpy context before spin() exits.
        # Treat that lifecycle event as a normal detector shutdown instead of
        # reporting a misleading traceback and exit code 1.
        pass
    except Exception:
        # Humble exposes some shutdown RCLError instances only from the
        # private C extension, so they cannot be imported portably.  Suppress
        # them only after the context is already invalid; preserve real
        # detector failures while ROS is running.
        if rclpy.ok():
            raise
    finally:
        node.close_debug_display()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

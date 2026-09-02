#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from visual_pick_to_bin import VisualPickToBin


class TargetCountMonitor(Node):

    def __init__(self):
        super().__init__('target_count_monitor')

        self.count = None
        self.last_msg_time = None

        self.create_subscription(
            Int32,
            '/vision/target_count',
            self.count_cb,
            10
        )

    def count_cb(self, msg):
        self.count = int(msg.data)
        self.last_msg_time = time.monotonic()

    def wait_for_stable_count(
        self,
        stable_time=1.2,
        timeout=15.0
    ):
        """
        Wait until /vision/target_count stays unchanged
        for stable_time seconds.

        This prevents the previous frame from being reused
        immediately after a cube has been moved.
        """

        start = time.monotonic()

        previous = None
        stable_since = None

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.10
            )

            now = time.monotonic()

            if now - start > timeout:
                raise RuntimeError(
                    'No stable /vision/target_count received'
                )

            if self.count is None:
                continue

            if self.count != previous:
                previous = self.count
                stable_since = now
                continue

            if stable_since is None:
                stable_since = now
                continue

            if now - stable_since >= stable_time:
                return self.count


def main(args=None):

    rclpy.init(args=args)

    monitor = TargetCountMonitor()

    picked = 0

    # Three previously verified safe bin lanes.
    # Avoid the old y=-0.986 configuration.
    place_y_positions = [
        -0.806,
        -0.866,
        -0.926,
    ]

    print()
    print('=' * 60)
    print('AUTO MULTI-OBJECT VISUAL PICK')
    print('Detect -> Pick -> Place -> Re-detect')
    print('=' * 60)

    try:

        while rclpy.ok():

            # -------------------------------------------------
            # Wait for the UPDATED scene.
            # Important: do not use the previous target pose.
            # -------------------------------------------------

            count = monitor.wait_for_stable_count(
                stable_time=1.2,
                timeout=15.0
            )

            print()
            print(
                f'[SCENE] targets remaining on table = {count}'
            )

            # -------------------------------------------------
            # Normal termination
            # -------------------------------------------------

            if count == 0:

                print()
                print('=' * 60)
                print('SOURCE TABLE EMPTY')
                print(f'TOTAL PICKED = {picked}')
                print('AUTO PICK COMPLETED')
                print('=' * 60)

                break

            # -------------------------------------------------
            # IMPORTANT:
            # Create a NEW picker every cycle.
            #
            # This prevents old /vision/target_pose messages
            # stored in the previous node from being reused.
            # -------------------------------------------------

            picker = VisualPickToBin()

            # Spread cubes between 3 known-safe bin positions.
            picker.place_x = -0.605
            picker.place_y = place_y_positions[
                picked % len(place_y_positions)
            ]

            picker.get_logger().info(
                '========================================'
            )

            picker.get_logger().info(
                f'MULTI PICK CYCLE {picked + 1}'
            )

            picker.get_logger().info(
                f'target_count={count}'
            )

            picker.get_logger().info(
                f'place=({picker.place_x:.3f}, '
                f'{picker.place_y:.3f})'
            )

            picker.get_logger().info(
                '========================================'
            )

            try:
                picker.run()

            except Exception:

                picker.destroy_node()
                raise

            picker.destroy_node()

            picked += 1

            # Allow:
            # - dropped cube to settle
            # - detector to see the new scene
            time.sleep(0.8)

    except KeyboardInterrupt:
        print('\nInterrupted.')

    except Exception as e:
        print()
        print(f'MULTI-OBJECT PICK FAILED: {e}')

    finally:

        monitor.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

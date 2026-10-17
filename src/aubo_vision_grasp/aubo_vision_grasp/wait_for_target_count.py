#!/usr/bin/env python3

"""
Wait reliably for a value on /vision/target_count.

This replaces repeated short-lived `ros2 topic echo --once` calls, which can
miss DDS discovery while Gazebo is under load.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class TargetCountWaiter(Node):

    def __init__(self):
        super().__init__('target_count_waiter')
        self.count = None
        self.create_subscription(
            Int32,
            '/vision/target_count',
            self.count_cb,
            10,
        )

    def count_cb(self, msg):
        self.count = msg.data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--expected', type=int, default=None)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = TargetCountWaiter()
    deadline = time.monotonic() + args.timeout
    last_reported = None

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

            if node.count is None:
                continue

            if node.count != last_reported:
                print(f'target_count={node.count}', flush=True)
                last_reported = node.count

            if args.expected is None or node.count == args.expected:
                return 0

            # A sample was received, but wait for the expected count.
            node.count = None

        if args.expected is None:
            print('ERROR: /vision/target_count did not publish.', flush=True)
        else:
            print(
                f'ERROR: did not receive target_count={args.expected} '
                f'within {args.timeout:.0f} seconds.',
                flush=True,
            )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

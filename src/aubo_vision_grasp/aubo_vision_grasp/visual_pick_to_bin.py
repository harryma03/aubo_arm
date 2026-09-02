#!/usr/bin/env python3

import time
import rclpy

from std_srvs.srv import Trigger

from auto_suction_grasp import AutoSuctionGrasp


class VisualPickToBin(AutoSuctionGrasp):

    def __init__(self):
        super().__init__()

        # 目前已经验证稳定的安全高度
        self.safe_z = 0.300

        # 小箱已验证可安全深入的位置
        self.place_x = -0.605
        self.place_y = -0.806

        # 分两段进入小箱
        self.place_entry_z = -0.050

        # 接近箱底再释放
        self.place_release_z = -0.150

        self.get_logger().info(
            'Visual random-cube -> small-bin node started'
        )

    def suction_off(self):

        if not self.suction_off_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError('/suction_off unavailable')

        future = self.suction_off_client.call_async(
            Trigger.Request()
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                'No /suction_off response'
            )

        self.get_logger().info(
            f'/suction_off: success={response.success}, '
            f'message="{response.message}"'
        )

        if not response.success:
            raise RuntimeError(
                response.message
            )

    def direct_or_ompl(self, name, x, y, z):

        self.get_logger().info(
            f'{name}: try shortest collision-checked Cartesian path'
        )

        try:
            trajectory = self.compute_cartesian(
                name,
                x,
                y,
                z
            )

        except Exception as e:

            self.get_logger().warn(
                f'{name}: direct Cartesian unavailable: {e}'
            )

            self.get_logger().warn(
                f'{name}: fallback to OMPL'
            )

            self.ompl_move_to(
                name + '_OMPL',
                x,
                y,
                z
            )

            return

        self.execute_trajectory(
            name,
            trajectory
        )

    def run(self):

        # =====================================================
        # 1. RGB-D target
        # =====================================================

        x, y, object_top_z = self.get_target()

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'VISUAL RANDOM PICK'
        )

        self.get_logger().info(
            f'target = '
            f'({x:.3f}, {y:.3f}, {object_top_z:.3f})'
        )

        self.get_logger().info(
            '========================================'
        )

        # 吸盘TCP在物体顶面约2cm上方时，
        # 当前Gazebo吸盘插件已经验证可以成功吸附。
        contact_z = object_top_z + 0.020

        # =====================================================
        # 2. 当前姿态 -> 目标上方
        # =====================================================

        self.get_logger().info(
            'STEP 1: shortest safe path -> PICK_SAFE'
        )

        self.direct_or_ompl(
            'PICK_SAFE',
            x,
            y,
            self.safe_z
        )

        time.sleep(0.3)

        # =====================================================
        # 3. 垂直下降
        # =====================================================

        self.get_logger().info(
            'STEP 2: Cartesian PICK DESCEND'
        )

        self.cartesian_move_to(
            'PICK_DESCEND',
            x,
            y,
            contact_z
        )

        time.sleep(0.3)

        # =====================================================
        # 4. 吸附
        # =====================================================

        self.get_logger().info(
            'STEP 3: SUCTION ON'
        )

        self.suction_on()

        time.sleep(0.4)

        # =====================================================
        # 5. 垂直抬升
        # =====================================================

        self.get_logger().info(
            'STEP 4: Cartesian PICK LIFT'
        )

        self.cartesian_move_to(
            'PICK_LIFT',
            x,
            y,
            self.safe_z
        )

        time.sleep(0.3)

        # =====================================================
        # 6. 最短安全路径 -> 小箱
        # =====================================================

        self.get_logger().info(
            'STEP 5: shortest safe TRANSFER'
        )

        self.direct_or_ompl(
            'TRANSFER',
            self.place_x,
            self.place_y,
            self.safe_z
        )

        time.sleep(0.3)

        # =====================================================
        # 7. 到箱口内部
        # =====================================================

        self.get_logger().info(
            'STEP 6: PLACE ENTRY'
        )

        self.cartesian_move_to(
            'PLACE_ENTRY',
            self.place_x,
            self.place_y,
            self.place_entry_z
        )

        time.sleep(0.2)

        # =====================================================
        # 8. 接近箱底
        # =====================================================

        self.get_logger().info(
            'STEP 7: PLACE FINAL DESCEND'
        )

        self.cartesian_move_to(
            'PLACE_DESCEND',
            self.place_x,
            self.place_y,
            self.place_release_z
        )

        time.sleep(0.3)

        # =====================================================
        # 9. 释放
        # =====================================================

        self.get_logger().info(
            'STEP 8: SUCTION OFF'
        )

        self.suction_off()

        # Release and leave promptly so the dropped object
        # does not bounce against the suction tool.
        time.sleep(0.20)

        # =====================================================
        # 10. Two-stage vertical retreat
        # =====================================================

        self.get_logger().info(
            'STEP 9A: PLACE RETREAT INNER'
        )

        self.cartesian_move_to(
            'PLACE_RETREAT_INNER',
            self.place_x,
            self.place_y,
            self.place_entry_z
        )

        time.sleep(0.15)

        self.get_logger().info(
            'STEP 9B: PLACE RETREAT SAFE'
        )

        self.cartesian_move_to(
            'PLACE_RETREAT_SAFE',
            self.place_x,
            self.place_y,
            self.safe_z
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'VISUAL PICK AND PLACE COMPLETED'
        )

        self.get_logger().info(
            '========================================'
        )


def main(args=None):

    rclpy.init(args=args)

    node = VisualPickToBin()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(
            f'VISUAL PICK FAILED: {e}'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

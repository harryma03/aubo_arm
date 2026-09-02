#!/usr/bin/env python3

import time
import rclpy

from std_srvs.srv import Trigger

# 与本文件在同一目录，直接导入已经跑通过的抓取基类
from auto_suction_grasp import AutoSuctionGrasp


class PlatformToBin(AutoSuctionGrasp):

    def __init__(self):
        super().__init__()

        self.declare_parameter('pick_only', True)

        self.get_logger().info(
            'Platform -> small bin pick-and-place started'
        )

    def suction_off(self):

        self.get_logger().info(
            'Calling /suction_off...'
        )

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
                'No response from /suction_off'
            )

        self.get_logger().info(
            f'/suction_off: success={response.success}, '
            f'message="{response.message}"'
        )

        if not response.success:
            raise RuntimeError(
                'Suction off failed: ' + response.message
            )

    def run(self):

        # =====================================================
        # 1. 当前测试红方块
        #
        # spawn:
        # x = 0.00
        # y = -0.90
        #
        # 当前大平台顶部约 z=-0.016
        # cube = 5 cm
        # cube top ≈ +0.034
        # =====================================================

        pick_x = 0.000
        pick_y = -0.900
        cube_top_z = 0.034

        # 抓取前安全高度
        pick_pre_z = 0.220

        # 之前验证过：
        # suction_tcp 在物体顶面约 +2 cm 可正常吸附
        pick_contact_z = cube_top_z + 0.020

        # 抓起来后的安全高度
        pick_lift_z = 0.300

        # =====================================================
        # 2. 小箱中心
        # 根据当前 workcell 几何位置
        # =====================================================

        place_x = -0.575
        place_y = -0.896

        # 跨越搬运高度
        place_pre_z = 0.300

        # 第一版先在小箱上方释放，让方块自然落入
        place_release_z = 0.100

        pick_only = bool(
            self.get_parameter('pick_only').value
        )

        self.get_logger().info(
            '======================================'
        )
        self.get_logger().info(
            'PLATFORM -> SMALL BIN'
        )
        self.get_logger().info(
            f'PICK: ({pick_x:.3f}, '
            f'{pick_y:.3f}, '
            f'{cube_top_z:.3f})'
        )
        self.get_logger().info(
            f'PLACE: ({place_x:.3f}, '
            f'{place_y:.3f})'
        )
        self.get_logger().info(
            f'pick_only={pick_only}'
        )
        self.get_logger().info(
            '======================================'
        )

        # =====================================================
        # STEP 1
        # OMPL 到红块正上方
        # =====================================================

        self.get_logger().info(
            'STEP 1: OMPL -> PICK_PRE'
        )

        self.ompl_move_to(
            'PICK_PRE',
            pick_x,
            pick_y,
            pick_pre_z
        )

        time.sleep(0.5)

        # =====================================================
        # STEP 2
        # 垂直下降
        # =====================================================

        self.get_logger().info(
            'STEP 2: Cartesian PICK_DESCEND'
        )

        self.cartesian_move_to(
            'PICK_DESCEND',
            pick_x,
            pick_y,
            pick_contact_z
        )

        time.sleep(0.5)

        # =====================================================
        # STEP 3
        # 开吸盘
        # =====================================================

        self.get_logger().info(
            'STEP 3: SUCTION ON'
        )

        self.suction_on()

        time.sleep(0.5)

        # =====================================================
        # STEP 4
        # 垂直抬起
        # =====================================================

        self.get_logger().info(
            'STEP 4: Cartesian PICK_LIFT'
        )

        self.cartesian_move_to(
            'PICK_LIFT',
            pick_x,
            pick_y,
            pick_lift_z
        )

        time.sleep(0.8)

        # =====================================================
        # 第一轮先只抓起来
        # =====================================================

        if pick_only:

            self.get_logger().info(
                '======================================'
            )
            self.get_logger().info(
                'PICK TEST COMPLETED'
            )
            self.get_logger().info(
                'Cube should now be attached.'
            )
            self.get_logger().info(
                '======================================'
            )

            return

        # =====================================================
        # STEP 5
        # OMPL 搬运到小箱上方
        # =====================================================

        self.get_logger().info(
            'STEP 5: OMPL -> PLACE_PRE'
        )

        self.ompl_move_to(
            'PLACE_PRE',
            place_x,
            place_y,
            place_pre_z
        )

        time.sleep(0.5)

        # =====================================================
        # STEP 6
        # 垂直下降到小箱
        # =====================================================

        self.get_logger().info(
            'STEP 6: Cartesian PLACE_DESCEND'
        )

        self.cartesian_move_to(
            'PLACE_DESCEND',
            place_x,
            place_y,
            place_release_z
        )

        time.sleep(0.5)

        # =====================================================
        # STEP 7
        # 松开
        # =====================================================

        self.get_logger().info(
            'STEP 7: SUCTION OFF'
        )

        self.suction_off()

        time.sleep(1.0)

        # =====================================================
        # STEP 8
        # 垂直退出小箱
        # =====================================================

        self.get_logger().info(
            'STEP 8: Cartesian RETREAT'
        )

        self.cartesian_move_to(
            'PLACE_RETREAT',
            place_x,
            place_y,
            place_pre_z
        )

        self.get_logger().info(
            '======================================'
        )
        self.get_logger().info(
            'PICK AND PLACE COMPLETED'
        )
        self.get_logger().info(
            '======================================'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PlatformToBin()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(
            f'FAILED: {e}'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

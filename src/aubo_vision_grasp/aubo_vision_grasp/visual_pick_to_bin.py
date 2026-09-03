#!/usr/bin/env python3

import time

import rclpy

from std_srvs.srv import Trigger

from auto_suction_grasp import AutoSuctionGrasp


class VisualPickToBin(AutoSuctionGrasp):

    def __init__(self):
        super().__init__()

        # =====================================================
        # SOURCE HOPPER
        # =====================================================

        # 料斗外部安全高度
        self.safe_z = 0.300

        # 料斗顶部约 z=-0.0156
        # 使用 +0.050 作为进入/退出料斗的公共高度
        self.hopper_entry_z = 0.050

        # 抓取前在目标上方 12 cm 停一次
        self.pregrasp_offset = 0.030

        # 当前吸盘插件已经验证：
        # TCP 在目标顶面约 2 cm 上方可吸附
        self.contact_offset = 0.020

        # =====================================================
        # RECEIVING BIN
        # =====================================================

        self.place_x = -0.605
        self.place_y = -0.806

        self.place_entry_z = -0.050
        self.place_release_z = -0.150

        self.get_logger().info(
            'Visual deep-hopper -> small-bin node started'
        )

    # =========================================================
    # Suction OFF
    # =========================================================

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

    # =========================================================
    # Direct Cartesian first, OMPL fallback
    # =========================================================

    def direct_or_ompl(
        self,
        name,
        x,
        y,
        z
    ):

        self.get_logger().info(
            f'{name}: try shortest collision-checked '
            'Cartesian path'
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

    # =========================================================
    # Main pick-place sequence
    # =========================================================

    def run(self):

        suction_active = False

        try:

            # =================================================
            # 1. RGB-D target
            # =================================================

            x, y, object_top_z = self.get_target()

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'VISUAL DEEP-HOPPER PICK'
            )

            self.get_logger().info(
                f'target = '
                f'({x:.3f}, {y:.3f}, {object_top_z:.3f})'
            )

            self.get_logger().info(
                '========================================'
            )

            # -------------------------------------------------
            # Basic target safety check
            #
            # Hopper bottom inner region is approximately:
            # x = +/-0.3075
            # y = -0.900 +/-0.465
            #
            # Leave a safety margin from walls.
            # -------------------------------------------------

            if not (
                -0.29 < x < 0.29 and
                -1.34 < y < -0.46 and
                -0.82 < object_top_z < 0.05
            ):
                raise RuntimeError(
                    'Target is outside safe deep-hopper '
                    f'pick region: '
                    f'({x:.3f}, {y:.3f}, {object_top_z:.3f})'
                )

            # -------------------------------------------------
            # Actual suction contact point
            # -------------------------------------------------

            contact_z = (
                object_top_z
                + self.contact_offset
            )

            # -------------------------------------------------
            # Intermediate pre-grasp point
            #
            # Example:
            # object top  = -0.776
            # pregrasp    = -0.656
            # contact     = -0.756
            # -------------------------------------------------

            pregrasp_z = (
                object_top_z
                + self.pregrasp_offset
            )

            # Never allow the pregrasp point to exceed
            # the hopper entry height.
            pregrasp_z = min(
                pregrasp_z,
                self.hopper_entry_z
            )

            if pregrasp_z <= contact_z:
                raise RuntimeError(
                    'Invalid pregrasp/contact geometry'
                )

            self.get_logger().info(
                f'hopper entry z = '
                f'{self.hopper_entry_z:.3f}'
            )

            self.get_logger().info(
                f'pregrasp z = {pregrasp_z:.3f}'
            )

            self.get_logger().info(
                f'contact z = {contact_z:.3f}'
            )

            # =================================================
            # 2. Move above selected target
            # =================================================

            self.get_logger().info(
                'STEP 1: -> PICK_SAFE'
            )

            self.direct_or_ompl(
                'PICK_SAFE',
                x,
                y,
                self.safe_z
            )

            time.sleep(0.3)

            # =================================================
            # 3. Move to hopper opening
            # =================================================

            self.get_logger().info(
                'STEP 2: Cartesian -> HOPPER_ENTRY'
            )

            self.cartesian_move_to(
                'PICK_ENTRY',
                x,
                y,
                self.hopper_entry_z
            )

            time.sleep(0.2)

            # =================================================
            # 4. Descend deep into hopper
            # =================================================

            self.get_logger().info(
                'STEP 3: Cartesian -> PICK_PREGRASP'
            )

            self.direct_or_ompl(
                'PICK_PREGRASP',
                x,
                y,
                pregrasp_z
            )

            time.sleep(0.2)

            # =================================================
            # 5. Final suction approach
            # =================================================

            self.get_logger().info(
                'STEP 4: Cartesian -> PICK_DESCEND'
            )

            self.cartesian_move_to(
                'PICK_DESCEND',
                x,
                y,
                contact_z
            )

            time.sleep(0.3)

            # =================================================
            # 6. Suction ON
            # =================================================

            self.get_logger().info(
                'STEP 5: SUCTION ON'
            )

            self.suction_on()

            suction_active = True

            time.sleep(0.4)

            # =================================================
            # 7. First lift inside hopper
            # =================================================

            self.get_logger().info(
                'STEP 6: Cartesian -> PICK_PRELIFT'
            )

            self.cartesian_move_to(
                'PICK_PRELIFT',
                x,
                y,
                pregrasp_z
            )

            time.sleep(0.2)

            # =================================================
            # 8. Exit hopper vertically
            # =================================================

            self.get_logger().info(
                'STEP 7: Cartesian -> PICK_EXIT'
            )

            self.cartesian_move_to(
                'PICK_EXIT',
                x,
                y,
                self.hopper_entry_z
            )

            time.sleep(0.2)

            # =================================================
            # 9. Lift to global safe height
            # =================================================

            self.get_logger().info(
                'STEP 8: Cartesian -> PICK_LIFT'
            )

            self.cartesian_move_to(
                'PICK_LIFT',
                x,
                y,
                self.safe_z
            )

            time.sleep(0.3)

            # =================================================
            # 10. Transfer to receiving bin
            # =================================================

            self.get_logger().info(
                'STEP 9: shortest safe TRANSFER'
            )

            self.direct_or_ompl(
                'TRANSFER',
                self.place_x,
                self.place_y,
                self.safe_z
            )

            time.sleep(0.3)

            # =================================================
            # 11. Enter receiving bin
            # =================================================

            self.get_logger().info(
                'STEP 10: PLACE ENTRY'
            )

            self.cartesian_move_to(
                'PLACE_ENTRY',
                self.place_x,
                self.place_y,
                self.place_entry_z
            )

            time.sleep(0.2)

            # =================================================
            # 12. Final place descent
            # =================================================

            self.get_logger().info(
                'STEP 11: PLACE FINAL DESCEND'
            )

            self.cartesian_move_to(
                'PLACE_DESCEND',
                self.place_x,
                self.place_y,
                self.place_release_z
            )

            time.sleep(0.3)

            # =================================================
            # 13. Release
            # =================================================

            self.get_logger().info(
                'STEP 12: SUCTION OFF'
            )

            self.suction_off()

            suction_active = False

            time.sleep(0.20)

            # =================================================
            # 14. Retreat inside receiving bin
            # =================================================

            self.get_logger().info(
                'STEP 13: PLACE RETREAT INNER'
            )

            self.cartesian_move_to(
                'PLACE_RETREAT_INNER',
                self.place_x,
                self.place_y,
                self.place_entry_z
            )

            time.sleep(0.2)

            # =================================================
            # 15. Return to global safe height
            # =================================================

            self.get_logger().info(
                'STEP 14: PLACE RETREAT SAFE'
            )

            self.cartesian_move_to(
                'PLACE_RETREAT_SAFE',
                self.place_x,
                self.place_y,
                self.safe_z
            )

            time.sleep(0.3)

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'VISUAL PICK AND PLACE COMPLETED'
            )

            self.get_logger().info(
                '========================================'
            )

            return True

        except Exception as e:

            self.get_logger().error(
                '========================================'
            )

            self.get_logger().error(
                f'VISUAL PICK FAILED: {e}'
            )

            self.get_logger().error(
                '========================================'
            )

            # If something fails while carrying an object,
            # attempt to release the suction safely.
            if suction_active:

                try:
                    self.suction_off()

                except Exception as suction_error:

                    self.get_logger().error(
                        f'Failed to turn suction off '
                        f'after error: {suction_error}'
                    )

            raise


def main(args=None):

    rclpy.init(args=args)

    node = VisualPickToBin()

    try:

        node.run()

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

#!/usr/bin/env python3

import copy
import math
import time
from collections import deque

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


MOVEIT_SUCCESS = 1


class AutoSuctionGrasp(Node):

    def __init__(self):
        super().__init__('auto_suction_grasp')

        self.samples = deque(maxlen=20)
        self.latest_joint_state = None

        # -------------------------
        # Subscriptions
        # -------------------------
        self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_cb,
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            20
        )

        # -------------------------
        # MoveIt
        # -------------------------
        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        # -------------------------
        # Suction
        # -------------------------
        self.suction_on_client = self.create_client(
            Trigger,
            '/suction_on'
        )

        self.suction_off_client = self.create_client(
            Trigger,
            '/suction_off'
        )

        self.get_logger().info(
            'Automatic suction grasp node started '
            '(OMPL + Cartesian descent/lift)'
        )

    # ==========================================================
    # Sensor callbacks
    # ==========================================================

    def target_cb(self, msg):
        if msg.header.frame_id != 'base_link':
            return

        p = msg.pose.position

        xyz = np.array([
            p.x,
            p.y,
            p.z
        ], dtype=float)

        if np.all(np.isfinite(xyz)):
            self.samples.append(xyz)

    def joint_state_cb(self, msg):
        self.latest_joint_state = copy.deepcopy(msg)

    # ==========================================================
    # Vision target
    # ==========================================================

    def get_target(self):

        self.get_logger().info(
            'Waiting for stable visual target...'
        )

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if len(self.samples) >= 10:
                break

            if time.time() - start > 10.0:
                raise RuntimeError(
                    'No stable /vision/target_pose received'
                )

        data = np.array(self.samples)

        target = np.median(data, axis=0)

        x, y, z = target

        self.get_logger().info(
            f'Visual target: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}'
        )

        # 防止错误识别后机械臂乱跑
        # Current workcell tabletop safe ROI.
        # z is the detected object TOP surface.
        if not (
            -0.30 < x < 0.30 and
            -1.10 < y < -0.47 and
            -0.02 < z < 0.10
        ):
            raise RuntimeError(
                f'Target outside safe grasp workspace: '
                f'({x:.3f}, {y:.3f}, {z:.3f})'
            )

        return x, y, z

    # ==========================================================
    # Common downward suction pose
    # ==========================================================

    @staticmethod
    def make_downward_pose(x, y, z):

        pose = Pose()

        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        # suction_tcp 朝下
        # RPY = (pi, 0, 0)
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    # ==========================================================
    # OMPL pose planning
    # ==========================================================

    def make_move_group_goal(self, x, y, z):

        goal = MoveGroup.Goal()

        req = goal.request

        req.group_name = 'manipulator'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15

        req.start_state.is_diff = True

        constraint = Constraints()
        constraint.name = 'suction_tcp_goal'

        # -------------------------
        # Position
        # -------------------------

        pc = PositionConstraint()

        pc.header.frame_id = 'base_link'
        pc.link_name = 'suction_tcp'
        pc.weight = 1.0

        region = BoundingVolume()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE

        # 8 mm tolerance
        sphere.dimensions = [0.008]

        sphere_pose = Pose()
        sphere_pose.position.x = float(x)
        sphere_pose.position.y = float(y)
        sphere_pose.position.z = float(z)
        sphere_pose.orientation.w = 1.0

        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)

        pc.constraint_region = region

        # -------------------------
        # Orientation
        # -------------------------

        oc = OrientationConstraint()

        oc.header.frame_id = 'base_link'
        oc.link_name = 'suction_tcp'

        oc.orientation.x = 1.0
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0

        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15

        # 吸盘绕自身轴旋转不重要
        # Keep suction TCP orientation deterministic.
        # Previously this was math.pi, allowing OMPL to choose
        # very different wrist/IK configurations at the same pose.
        oc.absolute_z_axis_tolerance = math.pi

        oc.weight = 1.0

        constraint.position_constraints.append(pc)
        constraint.orientation_constraints.append(oc)

        # --------------------------------------------------
        # Keep wrist joints away from the +/- 2*pi limits.
        #
        # Without these constraints, the same TCP pose can be
        # represented by e.g.
        #
        #   wrist3 = +0.35 rad
        #
        # or an equivalent wrapped solution near
        #
        #   wrist3 = -5.93 rad
        #
        # The latter leaves almost no joint margin for the
        # subsequent Cartesian vertical motion.
        #
        # Keep both wrist1 and wrist3 within approximately
        # [-pi, +pi].
        # --------------------------------------------------

        wrist1_constraint = JointConstraint()
        wrist1_constraint.joint_name = 'wrist1_joint'
        wrist1_constraint.position = 0.0
        wrist1_constraint.tolerance_above = math.pi
        wrist1_constraint.tolerance_below = math.pi
        wrist1_constraint.weight = 1.0

        wrist3_constraint = JointConstraint()
        wrist3_constraint.joint_name = 'wrist3_joint'
        wrist3_constraint.position = 0.0
        wrist3_constraint.tolerance_above = math.pi
        wrist3_constraint.tolerance_below = math.pi
        wrist3_constraint.weight = 1.0

        constraint.joint_constraints.append(
            wrist1_constraint
        )

        constraint.joint_constraints.append(
            wrist3_constraint
        )

        req.goal_constraints.append(constraint)

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def ompl_move_to(self, name, x, y, z):

        self.get_logger().info(
            f'{name}: '
            f'({x:.3f}, {y:.3f}, {z:.3f})'
        )

        if not self.move_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/move_action unavailable'
            )

        goal = self.make_move_group_goal(
            x, y, z
        )

        send_future = self.move_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        handle = send_future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(
                f'{name}: MoveIt goal rejected'
            )

        result_future = handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError(
                f'{name}: no MoveIt result'
            )

        result = wrapped.result

        if result.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f'{name}: MoveIt failed, '
                f'error_code={result.error_code.val}'
            )

        self.get_logger().info(
            f'{name}: SUCCESS'
        )

    # ==========================================================
    # Cartesian planning
    # ==========================================================

    def wait_for_joint_state(self):

        start = time.time()

        while rclpy.ok() and self.latest_joint_state is None:

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if time.time() - start > 5.0:
                raise RuntimeError(
                    'No /joint_states received'
                )

    def compute_cartesian(self, name, x, y, z):

        self.wait_for_joint_state()

        if not self.cartesian_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/compute_cartesian_path unavailable'
            )

        req = GetCartesianPath.Request()

        req.header.frame_id = 'base_link'
        req.group_name = 'manipulator'
        req.link_name = 'suction_tcp'

        start_state = RobotState()

        start_state.joint_state = copy.deepcopy(
            self.latest_joint_state
        )

        start_state.is_diff = False

        req.start_state = start_state

        waypoint = self.make_downward_pose(
            x, y, z
        )

        req.waypoints.append(waypoint)

        # 每 5 mm 插值一次
        req.max_step = 0.003

        # 关闭旧式 jump threshold
        req.jump_threshold = 0.0

        # 必须检查自碰撞/Planning Scene碰撞
        req.avoid_collisions = True

        self.get_logger().info(
            f'{name}: requesting Cartesian path '
            f'to ({x:.3f}, {y:.3f}, {z:.3f})'
        )

        future = self.cartesian_client.call_async(
            req
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f'{name}: no Cartesian response'
            )

        self.get_logger().info(
            f'{name}: Cartesian fraction='
            f'{response.fraction:.3f}'
        )

        if response.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f'{name}: Cartesian planning failed, '
                f'error_code={response.error_code.val}'
            )

        if response.fraction < 0.98:
            raise RuntimeError(
                f'{name}: Cartesian path incomplete, '
                f'fraction={response.fraction:.3f}'
            )

        # --------------------------------------------------
        # Absolute joint-jump guard
        #
        # MoveIt's relative jump_threshold can be too
        # conservative near some AUBO configurations.
        #
        # We therefore keep jump_threshold=0.0, but reject
        # any trajectory containing a large instantaneous
        # joint change.
        #
        # Collision checking is still handled by:
        #     req.avoid_collisions = True
        # --------------------------------------------------

        points = response.solution.joint_trajectory.points

        max_joint_step = 0.0

        if len(points) >= 2:

            for prev, curr in zip(points[:-1], points[1:]):

                for a, b in zip(
                    prev.positions,
                    curr.positions
                ):
                    # shortest angular difference
                    delta = abs(
                        (b - a + math.pi) %
                        (2.0 * math.pi) -
                        math.pi
                    )

                    max_joint_step = max(
                        max_joint_step,
                        delta
                    )

        self.get_logger().info(
            f'{name}: max joint step='
            f'{max_joint_step:.4f} rad'
        )

        # 0.25 rad ≈ 14.3 deg between adjacent
        # Cartesian interpolation points.
        #
        # Anything larger is suspicious and is rejected
        # rather than allowing the arm to flip/wind around.
        if max_joint_step > 0.25:
            raise RuntimeError(
                f'{name}: excessive joint jump '
                f'{max_joint_step:.4f} rad'
            )

        return response.solution

    # ==========================================================
    # Execute RobotTrajectory
    # ==========================================================

    def execute_trajectory(self, name, trajectory):

        if not self.execute_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/execute_trajectory unavailable'
            )

        goal = ExecuteTrajectory.Goal()

        goal.trajectory = trajectory

        self.get_logger().info(
            f'{name}: executing Cartesian trajectory'
        )

        future = self.execute_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        handle = future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(
                f'{name}: execution rejected'
            )

        result_future = handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError(
                f'{name}: no execution result'
            )

        result = wrapped.result

        if result.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f'{name}: execution failed, '
                f'error_code={result.error_code.val}'
            )

        self.get_logger().info(
            f'{name}: SUCCESS'
        )

    def cartesian_move_to(self, name, x, y, z):

        trajectory = self.compute_cartesian(
            name,
            x,
            y,
            z
        )

        self.execute_trajectory(
            name,
            trajectory
        )

    # ==========================================================
    # Suction
    # ==========================================================

    def suction_on(self):

        self.get_logger().info(
            'Calling /suction_on...'
        )

        if not self.suction_on_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/suction_on unavailable'
            )

        future = self.suction_on_client.call_async(
            Trigger.Request()
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                'No /suction_on response'
            )

        self.get_logger().info(
            f'/suction_on: '
            f'success={response.success}, '
            f'message="{response.message}"'
        )

        if not response.success:
            raise RuntimeError(
                'Suction failed: '
                + response.message
            )

    # ==========================================================
    # Automatic grasp sequence
    # ==========================================================

    def run(self):

        x, y, object_z = self.get_target()

        # 目标视觉得到的是方块顶面附近
        pre_z = object_z + 0.15

        # 已经验证过这个高度可以吸附
        suction_z = object_z + 0.02

        # 抬到目标上方 22 cm
        lift_z = object_z + 0.22

        self.get_logger().info(
            '===================================='
        )
        self.get_logger().info(
            'AUTO GRASP SEQUENCE'
        )
        self.get_logger().info(
            '===================================='
        )

        # ------------------------------------------------------
        # 1. 全局规划到方块上方
        # ------------------------------------------------------

        self.get_logger().info(
            'STEP 1/4: OMPL -> PRE_GRASP'
        )

        self.ompl_move_to(
            'PRE_GRASP',
            x,
            y,
            pre_z
        )

        time.sleep(0.5)

        # ------------------------------------------------------
        # 2. 只沿 Z 垂直下降
        # ------------------------------------------------------

        self.get_logger().info(
            'STEP 2/4: Cartesian vertical DESCEND'
        )

        self.cartesian_move_to(
            'DESCEND',
            x,
            y,
            suction_z
        )

        time.sleep(0.5)

        # ------------------------------------------------------
        # 3. 吸附
        # ------------------------------------------------------

        self.get_logger().info(
            'STEP 3/4: SUCTION ON'
        )

        self.suction_on()

        time.sleep(0.5)

        # ------------------------------------------------------
        # 4. 只沿 Z 垂直抬升
        # ------------------------------------------------------

        self.get_logger().info(
            'STEP 4/4: Cartesian vertical LIFT'
        )

        self.cartesian_move_to(
            'LIFT',
            x,
            y,
            lift_z
        )

        self.get_logger().info(
            '===================================='
        )
        self.get_logger().info(
            'AUTO GRASP COMPLETED'
        )
        self.get_logger().info(
            '===================================='
        )


def main(args=None):

    rclpy.init(args=args)

    node = AutoSuctionGrasp()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(
            f'AUTO GRASP FAILED: {e}'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

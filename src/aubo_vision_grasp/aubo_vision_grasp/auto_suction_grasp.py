#!/usr/bin/env python3

import copy
import math
import time
from collections import deque

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionIK,
    GetStateValidity,
)
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
    JointConstraint,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

try:
    from .system_config import load_system_config
except ImportError:
    from system_config import load_system_config


MOVEIT_SUCCESS = 1


class AutoSuctionGrasp(Node):

    MOVEIT_ERROR_NAMES = {
        MoveItErrorCodes.SUCCESS: 'SUCCESS',
        MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
        MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
        MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
        MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
        MoveItErrorCodes.START_STATE_IN_COLLISION:
            'START_STATE_IN_COLLISION',
        MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            'GOAL_CONSTRAINTS_VIOLATED',
        MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
    }

    MANIPULATOR_JOINTS = (
        'shoulder_joint',
        'upperArm_joint',
        'foreArm_joint',
        'wrist1_joint',
        'wrist2_joint',
        'wrist3_joint',
    )

    def __init__(
        self,
        node_name='auto_suction_grasp',
        *,
        use_global_arguments=True,
    ):
        # Nodes created inside visual_pick_queue must ignore the process-wide
        # ``__node:=visual_pick_queue`` remap or every internal executor gets
        # the same ROS name and rosout publisher.
        super().__init__(
            node_name,
            use_global_arguments=use_global_arguments,
        )

        self.system_config = load_system_config()
        workcell = self.system_config.workcell
        self.workcell_x = workcell.base_x
        self.workcell_y = workcell.base_y
        self.workcell_z = workcell.z
        self.workcell_dz = workcell.dz
        self.hopper_x = workcell.hopper_x
        self.hopper_y = workcell.hopper_y
        self.hopper_top_z = workcell.hopper_top_z
        self.hopper_dz = workcell.hopper_dz

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

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
        )

        self.state_validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
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
        # Current deep-hopper safe ROI.
        # z is the detected object TOP surface.
        if not (
            self.hopper_x - 0.36 < x < self.hopper_x + 0.36 and
            self.hopper_y - 0.53 < y < self.hopper_y + 0.53 and
            -0.82 + self.hopper_dz < z < 0.10 + self.hopper_dz
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
    def make_downward_pose(x, y, z, yaw=0.0):

        pose = Pose()

        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        # suction_tcp downward, RPY=(pi, 0, yaw).  Explicit yaw candidates
        # let the planner choose a reachable wrist branch without permitting
        # an uncontrolled 180-degree flip.
        pose.orientation.x = math.cos(float(yaw) * 0.5)
        pose.orientation.y = math.sin(float(yaw) * 0.5)
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    # ==========================================================
    # OMPL pose planning
    # ==========================================================

    def make_move_group_goal(
        self,
        x,
        y,
        z,
        *,
        yaw=0.0,
        plan_only=False,
        start_state=None,
    ):

        goal = MoveGroup.Goal()

        req = goal.request

        req.group_name = 'manipulator'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = (
            self.system_config.motion.ompl_planning_attempts
        )
        req.allowed_planning_time = (
            self.system_config.motion.ompl_planning_time
        )

        req.max_velocity_scaling_factor = (
            self.system_config.motion.velocity_scaling
        )
        req.max_acceleration_scaling_factor = (
            self.system_config.motion.acceleration_scaling
        )

        if start_state is None:
            req.start_state.is_diff = True
        else:
            req.start_state = copy.deepcopy(start_state)

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

        downward_pose = self.make_downward_pose(x, y, z, yaw=yaw)
        oc.orientation = downward_pose.orientation

        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15

        # Each candidate represents a deliberate wrist branch.  A narrow yaw
        # tolerance prevents different candidates collapsing into the same
        # solution while still absorbing numerical IK variation.
        oc.absolute_z_axis_tolerance = 0.18

        oc.weight = 1.0

        constraint.position_constraints.append(pc)
        constraint.orientation_constraints.append(oc)

        req.goal_constraints.append(constraint)

        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def make_joint_state_goal(self, joint_values, tolerance=None):
        """Build a collision-checked goal for the SRDF ``up`` pose."""
        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = 'manipulator'
        req.num_planning_attempts = (
            self.system_config.motion.ompl_planning_attempts
        )
        req.allowed_planning_time = (
            self.system_config.motion.ompl_planning_time
        )
        req.max_velocity_scaling_factor = (
            self.system_config.motion.velocity_scaling
        )
        req.max_acceleration_scaling_factor = (
            self.system_config.motion.acceleration_scaling
        )
        req.start_state.is_diff = True

        if tolerance is None:
            tolerance = self.system_config.motion.recovery_joint_tolerance
        constraint = Constraints()
        constraint.name = 'recovery_up_pose'

        for joint_name, position in joint_values.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraint.joint_constraints.append(joint_constraint)

        req.goal_constraints.append(constraint)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _request_move_group(self, name, goal):
        if not self.move_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/move_action unavailable'
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
            error_name = self.moveit_error_name(result.error_code.val)
            raise RuntimeError(
                f'{name}: MoveIt failed, '
                f'error_code={result.error_code.val} ({error_name})'
            )

        return result

    @classmethod
    def moveit_error_name(cls, value):
        return cls.MOVEIT_ERROR_NAMES.get(int(value), 'UNKNOWN_MOVEIT_ERROR')

    def diagnose_pose_goal(self, x, y, z, yaw=0.0):
        """Explain whether a failed goal lacks IK or is in collision."""
        if not self.ik_client.wait_for_service(timeout_sec=0.5):
            return 'diagnostic unavailable: /compute_ik is not ready'

        self.wait_for_joint_state()
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = 'manipulator'
        ik_request.ik_request.ik_link_name = 'suction_tcp'
        ik_request.ik_request.avoid_collisions = False
        ik_request.ik_request.robot_state.joint_state = copy.deepcopy(
            self.latest_joint_state
        )
        ik_request.ik_request.robot_state.is_diff = False
        ik_request.ik_request.pose_stamped.header.frame_id = 'base_link'
        ik_request.ik_request.pose_stamped.pose = self.make_downward_pose(
            x,
            y,
            z,
            yaw=yaw,
        )
        ik_request.ik_request.timeout = Duration(
            sec=0,
            nanosec=500_000_000,
        )

        future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        response = future.result()
        if response is None:
            return 'diagnostic unavailable: /compute_ik timed out'
        if response.error_code.val != MOVEIT_SUCCESS:
            return (
                'goal IK failed: '
                f'{self.moveit_error_name(response.error_code.val)}'
            )

        if not self.state_validity_client.wait_for_service(timeout_sec=0.5):
            return 'IK exists; /check_state_validity is not ready'

        validity_request = GetStateValidity.Request()
        validity_request.robot_state = response.solution
        validity_request.group_name = 'manipulator'
        future = self.state_validity_client.call_async(validity_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        validity = future.result()
        if validity is None:
            return 'IK exists; state-validity diagnostic timed out'
        if validity.valid:
            return (
                'goal IK is collision-free; failure is in the connecting '
                'path or goal tolerances'
            )

        pairs = []
        for contact in validity.contacts:
            pair = (
                f'{contact.contact_body_1}<->{contact.contact_body_2}'
            )
            if pair not in pairs:
                pairs.append(pair)
            if len(pairs) >= 4:
                break
        if pairs:
            return 'goal collision: ' + ', '.join(pairs)
        return 'goal state is invalid; MoveIt returned no contact pair'

    def plan_ompl_to(
        self,
        name,
        x,
        y,
        z,
        *,
        yaw=0.0,
        start_state=None,
    ):
        """Plan a pose goal without moving the robot."""
        self.get_logger().info(
            f'{name}: preflight ({x:.3f}, {y:.3f}, {z:.3f}), '
            f'yaw={math.degrees(yaw):+.1f}deg'
        )
        result = self._request_move_group(
            name,
            self.make_move_group_goal(
                x,
                y,
                z,
                yaw=yaw,
                plan_only=True,
                start_state=start_state,
            ),
        )
        trajectory = result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError(f'{name}: MoveIt returned an empty trajectory')
        self.get_logger().info(f'{name}: PREFLIGHT SUCCESS')
        return trajectory

    def ompl_move_to(self, name, x, y, z, *, yaw=0.0):
        self.get_logger().info(
            f'{name}: ({x:.3f}, {y:.3f}, {z:.3f})'
        )
        self._request_move_group(
            name,
            self.make_move_group_goal(x, y, z, yaw=yaw),
        )
        self.get_logger().info(f'{name}: SUCCESS')

    def ompl_move_to_joint_values(self, name, joint_values, tolerance=None):
        """Move to a joint pose through MoveIt's collision checks."""
        self.get_logger().info(f'{name}: planning recovery joint pose')

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/move_action unavailable for recovery')

        goal = self.make_joint_state_goal(joint_values, tolerance=tolerance)
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(f'{name}: MoveIt goal rejected')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError(f'{name}: no MoveIt result')

        result = wrapped.result
        if result.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f'{name}: MoveIt failed, '
                f'error_code={result.error_code.val}'
            )

        self.get_logger().info(f'{name}: SUCCESS')

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

    def current_manipulator_joint_values(self):
        """Return the latest six-joint pose in a deterministic name order."""
        self.wait_for_joint_state()
        rclpy.spin_once(self, timeout_sec=0.05)
        positions = dict(zip(
            self.latest_joint_state.name,
            self.latest_joint_state.position,
        ))
        missing = [
            name for name in self.MANIPULATOR_JOINTS
            if name not in positions
        ]
        if missing:
            raise RuntimeError(
                'Joint state is missing: ' + ', '.join(missing)
            )
        return {
            name: float(positions[name])
            for name in self.MANIPULATOR_JOINTS
        }

    def compute_cartesian(self, name, x, y, z, *, yaw=0.0):

        return self.compute_cartesian_waypoints(
            name,
            [(x, y, z)],
            yaw=yaw,
        )

    def compute_cartesian_waypoints(
        self,
        name,
        xyz_waypoints,
        *,
        start_state=None,
        yaw=0.0,
    ):

        if not xyz_waypoints:
            raise ValueError(f'{name}: Cartesian waypoint list is empty')

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

        if start_state is None:
            start_state = RobotState()
            start_state.joint_state = copy.deepcopy(self.latest_joint_state)
            start_state.is_diff = False
        req.start_state = copy.deepcopy(start_state)

        for x, y, z in xyz_waypoints:
            req.waypoints.append(
                self.make_downward_pose(x, y, z, yaw=yaw)
            )

        # 每 5 mm 插值一次
        req.max_step = 0.003

        # 关闭旧式 jump threshold
        req.jump_threshold = 0.0

        # 必须检查自碰撞/Planning Scene碰撞
        req.avoid_collisions = True

        self.get_logger().info(
            f'{name}: requesting Cartesian path through '
            f'{len(xyz_waypoints)} waypoint(s); final='
            f'({xyz_waypoints[-1][0]:.3f}, '
            f'{xyz_waypoints[-1][1]:.3f}, '
            f'{xyz_waypoints[-1][2]:.3f})'
        )

        future = self.cartesian_client.call_async(
            req
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done():
            future.cancel()
            raise RuntimeError(
                'Timed out waiting for /suction_on response'
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

    @staticmethod
    def robot_state_at_trajectory_end(trajectory):
        """Build an exact Cartesian start state from a planned trajectory."""
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError('Cannot derive state from an empty trajectory')
        state = RobotState()
        state.joint_state.name = list(joint_trajectory.joint_names)
        state.joint_state.position = list(
            joint_trajectory.points[-1].positions
        )
        state.is_diff = True
        return state

    @staticmethod
    def reverse_trajectory(trajectory):
        """Reverse a time-parameterized joint trajectory exactly."""
        reversed_trajectory = copy.deepcopy(trajectory)
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError('Cannot reverse an empty trajectory')

        def duration_ns(duration):
            return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)

        total_ns = duration_ns(points[-1].time_from_start)
        reversed_points = []
        for source in reversed(points):
            point = copy.deepcopy(source)
            elapsed_ns = total_ns - duration_ns(source.time_from_start)
            point.time_from_start.sec = elapsed_ns // 1_000_000_000
            point.time_from_start.nanosec = elapsed_ns % 1_000_000_000
            if point.velocities:
                point.velocities = [-value for value in point.velocities]
            reversed_points.append(point)

        reversed_trajectory.joint_trajectory.points = reversed_points
        return reversed_trajectory

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
            f'{name}: executing planned trajectory'
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

    def cartesian_move_to(self, name, x, y, z, *, yaw=0.0):

        trajectory = self.compute_cartesian(
            name,
            x,
            y,
            z,
            yaw=yaw,
        )

        self.execute_trajectory(
            name,
            trajectory
        )
        return trajectory

    def cartesian_move_through(self, name, xyz_waypoints, *, yaw=0.0):
        trajectory = self.compute_cartesian_waypoints(
            name,
            xyz_waypoints,
            yaw=yaw,
        )
        self.execute_trajectory(name, trajectory)
        return trajectory

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

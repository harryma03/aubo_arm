#!/usr/bin/env python3

import copy
import math
import time

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

try:
    from .auto_suction_grasp import AutoSuctionGrasp
except ImportError:
    from auto_suction_grasp import AutoSuctionGrasp


class PickFailure(RuntimeError):
    """A pick failure carrying a scheduler-safe reason code."""

    def __init__(self, reason, phase, detail):
        self.reason = str(reason)
        self.phase = str(phase)
        self.detail = str(detail)
        super().__init__(
            f'{self.reason} at {self.phase}: {self.detail}'
        )


class VisualPickToBin(AutoSuctionGrasp):

    # Try nearby wrist branches first.  Stop after three direct-transfer
    # candidates have succeeded, then choose the one with the least total
    # joint travel.  OMPL fallbacks do not stop the direct-route search.
    GRASP_YAW_CANDIDATES = tuple(math.radians(value) for value in (
        0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0,
    ))

    # An OMPL fallback is allowed only when its geometric path remains
    # reasonably local.  This rejects distant IK branches and full-joint
    # winding such as the long transfer observed in the place recording.
    MAX_OMPL_TRANSFER_JOINT_TRAVEL = 12.0

    def __init__(
        self,
        target_xyz=None,
        grasp_xy=None,
        staged_approach=False,
        node_name='auto_suction_grasp',
        bootstrap_ready=True,
        ready_joint_values=None,
        use_global_arguments=True,
    ):
        super().__init__(
            node_name=node_name,
            use_global_arguments=use_global_arguments,
        )

        # The dynamic queue passes an explicitly selected target.  Keeping
        # this optional preserves the legacy single-target entry point.
        self.target_xyz = None if target_xyz is None else tuple(
            float(value) for value in target_xyz
        )
        self.grasp_xy = None if grasp_xy is None else tuple(
            float(value) for value in grasp_xy
        )
        if self.grasp_xy is not None and len(self.grasp_xy) != 2:
            raise ValueError('grasp_xy must contain exactly two values')
        self.staged_approach = bool(staged_approach)

        # Geometry was loaded once by the base class from system_config.py.
        workcell = self.system_config.workcell
        self.safe_half_x = workcell.safe_half_x
        self.safe_half_y = workcell.safe_half_y

        # =====================================================
        # SOURCE HOPPER
        # =====================================================

        # 料斗外部安全高度
        self.safe_z = 0.300 + self.workcell_dz
        self.ready_x = self.hopper_x
        self.ready_y = self.hopper_y
        self.bootstrap_ready = bool(bootstrap_ready)
        self.ended_at_ready = False
        configured_ready = (
            ready_joint_values
            if ready_joint_values is not None
            else self.system_config.motion.ready_joint_values
        )
        if configured_ready is None:
            self.ready_joint_values = None
        elif isinstance(configured_ready, dict):
            self.ready_joint_values = dict(configured_ready)
        else:
            self.ready_joint_values = dict(zip(
                self.MANIPULATOR_JOINTS,
                configured_ready,
            ))
        self.captured_ready_joint_values = self.ready_joint_values

        # 料斗顶部约 z=-0.0156
        # 使用 +0.050 作为进入/退出料斗的公共高度
        self.hopper_entry_z = 0.050 + self.hopper_dz

        # A held cube must first clear the hopper rim vertically.  Only then
        # may it move laterally toward the receiving bin.  The corridor is on
        # the negative-X side, away from the overhead camera head near the
        # hopper centre.
        self.pick_clear_z = (
            self.hopper_entry_z
            + workcell.cube_z
            + 0.030
        )
        self.exit_corridor_x = (
            self.hopper_x
            - workcell.hopper_bottom_x * 0.5
            - 0.100
        )
        self.exit_corridor_y = self.hopper_y

        # 抓取前在目标上方 12 cm 停一次
        self.pregrasp_offset = 0.120

        # 当前吸盘插件已经验证：
        # TCP 在目标顶面约 2 cm 上方可吸附
        self.contact_offset = 0.020

        # =====================================================
        # RECEIVING BIN
        # =====================================================

        self.place_x = self.workcell_x - 0.605
        self.place_y = self.workcell_y - 0.806

        self.place_entry_z = -0.050 + self.workcell_dz
        self.place_release_z = -0.150 + self.workcell_dz

        # The receiving-bin walls top out at about -0.029 m in the default
        # workcell.  Keep the payload bottom 30 mm above that rim, while also
        # retaining the larger clearance already required to exit the hopper.
        receiving_bin_rim_z = -0.029 + self.workcell_dz
        payload_below_tcp = self.contact_offset + workcell.cube_z
        self.transfer_z = max(
            self.pick_clear_z,
            receiving_bin_rim_z + payload_below_tcp + 0.030,
        )

        # Failure recovery is deliberately conservative: a failed pick must
        # never leave the TCP at the contact point while the queue waits for
        # vision.  The timeout limits the total recovery attempt.
        self.recovery_timeout = self.system_config.motion.recovery_timeout
        self._phase = 'INIT'
        self._attached = False
        self._attached_collision = False
        self._pick_descent_trajectory = None
        self._pick_exit_trajectory = None
        self._transfer_lift_trajectory = None
        self._transfer_trajectory = None
        self._transfer_mode = None
        self._at_pick_contact = False
        self._at_pick_pregrasp = False
        self._at_pick_corridor = False
        self._payload_scene_id = 'held_grasp_cube'
        self.grasp_yaw = 0.0
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )

        self.get_logger().info(
            'Visual deep-hopper -> small-bin node started'
        )

    # =========================================================
    # Suction OFF
    # =========================================================

    def _make_attached_payload(self, target_xyz):
        """Build the MoveIt proxy for a cube whose top pose is in base_link."""
        attached = AttachedCollisionObject()
        attached.link_name = 'suction_tcp'
        attached.touch_links = [
            'suction_tcp',
            'suction_pad_link',
            'suction_cup_link',
            'ee_link',
            'wrist3_Link',
        ]
        attached.object.id = self._payload_scene_id
        attached.object.header.frame_id = 'base_link'
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        workcell = self.system_config.workcell
        # Use a slightly inset proxy.  The physical Gazebo cube remains
        # unchanged; this only absorbs RGB-D/contact-model millimetre errors.
        primitive.dimensions = [
            max(0.001, workcell.cube_x - 0.008),
            max(0.001, workcell.cube_y - 0.008),
            max(0.001, workcell.cube_z - 0.006),
        ]
        pose = Pose()
        pose.position.x = float(target_xyz[0])
        pose.position.y = float(target_xyz[1])
        pose.position.z = float(target_xyz[2]) - workcell.cube_z * 0.5
        pose.orientation.w = 1.0
        attached.object.primitives.append(primitive)
        attached.object.primitive_poses.append(pose)
        return attached

    def _apply_attached_payload(self, attach, target_xyz=None):
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/apply_planning_scene unavailable for payload')

        if attach:
            if target_xyz is None:
                raise RuntimeError('payload target pose is missing')
            attached = self._make_attached_payload(target_xyz)
        else:
            attached = AttachedCollisionObject()
            attached.link_name = 'suction_tcp'
            attached.object.id = self._payload_scene_id
            attached.object.header.frame_id = 'suction_tcp'
            attached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            action = 'attach' if attach else 'detach'
            raise RuntimeError(f'Failed to {action} payload in MoveIt scene')
        self._attached_collision = bool(attach)

    def _state_with_lifted_payload(
        self,
        robot_state,
        tcp_x,
        tcp_y,
        tcp_z,
        payload_offset=(0.0, 0.0),
    ):
        """Attach a predicted payload to a future Cartesian start state."""
        state = copy.deepcopy(robot_state)
        # At suction contact the TCP is contact_offset above the detected top.
        # The same relative offset remains after the object is lifted.
        lifted_top_z = float(tcp_z) - self.contact_offset
        state.attached_collision_objects.append(
            self._make_attached_payload((
                float(tcp_x) + float(payload_offset[0]),
                float(tcp_y) + float(payload_offset[1]),
                lifted_top_z,
            ))
        )
        state.is_diff = True
        return state

    @staticmethod
    def _trajectory_joint_travel(*trajectories):
        """Return a deterministic score for choosing the shortest candidate."""
        total = 0.0
        for trajectory in trajectories:
            points = trajectory.joint_trajectory.points
            for previous, current in zip(points[:-1], points[1:]):
                for left, right in zip(previous.positions, current.positions):
                    total += abs(
                        (right - left + math.pi) % (2.0 * math.pi) - math.pi
                    )
        return total

    def _plan_approach(self, label, x, y, pregrasp_z, yaw):
        """Plan either a direct or a hopper-centre staged approach."""
        if not self.staged_approach:
            return (
                self.plan_ompl_to(
                    f'PICK_APPROACH_{label}',
                    x,
                    y,
                    pregrasp_z,
                    yaw=yaw,
                ),
            )

        stage = self.plan_ompl_to(
            f'PICK_STAGE_{label}',
            self.hopper_x,
            self.hopper_y,
            self.safe_z,
            yaw=yaw,
        )
        stage_state = self.robot_state_at_trajectory_end(stage)
        approach = self.plan_ompl_to(
            f'PICK_APPROACH_FROM_STAGE_{label}',
            x,
            y,
            pregrasp_z,
            yaw=yaw,
            start_state=stage_state,
        )
        return stage, approach

    def _preflight_pick_candidates(
        self,
        x,
        y,
        pregrasp_z,
        contact_z,
        payload_offset=(0.0, 0.0),
    ):
        """Plan the complete loaded escape before moving toward a target."""
        feasible = []
        failures = []

        for yaw in self.GRASP_YAW_CANDIDATES:
            label = f'YAW_{math.degrees(yaw):+.0f}'
            try:
                approaches = self._plan_approach(
                    label,
                    x,
                    y,
                    pregrasp_z,
                    yaw,
                )
                pregrasp_state = self.robot_state_at_trajectory_end(
                    approaches[-1]
                )
                descent = self.compute_cartesian_waypoints(
                    f'PICK_DESCENT_{label}',
                    [(x, y, contact_z)],
                    start_state=pregrasp_state,
                    yaw=yaw,
                )

                loaded_pregrasp_state = self._state_with_lifted_payload(
                    pregrasp_state,
                    x,
                    y,
                    pregrasp_z,
                    payload_offset=payload_offset,
                )
                pick_exit = self.compute_cartesian_waypoints(
                    f'PICK_EXIT_{label}',
                    [
                        (x, y, self.pick_clear_z),
                        (
                            self.exit_corridor_x,
                            self.exit_corridor_y,
                            self.pick_clear_z,
                        ),
                    ],
                    start_state=loaded_pregrasp_state,
                    yaw=yaw,
                )

                corridor_state = self.robot_state_at_trajectory_end(pick_exit)
                loaded_corridor_state = self._state_with_lifted_payload(
                    corridor_state,
                    self.exit_corridor_x,
                    self.exit_corridor_y,
                    self.pick_clear_z,
                    payload_offset=payload_offset,
                )
                place_tcp_x = self.place_x - payload_offset[0]
                place_tcp_y = self.place_y - payload_offset[1]
                transfer_lift = None
                transfer_mode = 'CARTESIAN_LOW'

                try:
                    # The hopper exit is already above both container rims.
                    # Preserve the current IK branch with a short, direct,
                    # collision-checked Cartesian transfer at that height.
                    transfer = self.compute_cartesian_waypoints(
                        f'TRANSFER_CARTESIAN_{label}',
                        [(place_tcp_x, place_tcp_y, self.transfer_z)],
                        start_state=loaded_corridor_state,
                        yaw=yaw,
                    )
                except Exception as cartesian_error:
                    # Lift first when the low corridor is obstructed or
                    # crosses a singularity.  At the safe height, preserve the
                    # current IK branch with another Cartesian attempt before
                    # allowing OMPL to search for a less predictable route.
                    self.get_logger().warn(
                        f'{label}: low Cartesian transfer unavailable: '
                        f'{cartesian_error}; trying safe-lift + high '
                        'Cartesian fallback'
                    )
                    transfer_lift = self.compute_cartesian_waypoints(
                        f'TRANSFER_LIFT_{label}',
                        [(
                            self.exit_corridor_x,
                            self.exit_corridor_y,
                            self.safe_z,
                        )],
                        start_state=loaded_corridor_state,
                        yaw=yaw,
                    )
                    safe_corridor_state = self.robot_state_at_trajectory_end(
                        transfer_lift
                    )
                    loaded_safe_corridor_state = (
                        self._state_with_lifted_payload(
                            safe_corridor_state,
                            self.exit_corridor_x,
                            self.exit_corridor_y,
                            self.safe_z,
                            payload_offset=payload_offset,
                        )
                    )
                    try:
                        transfer_mode = 'CARTESIAN_HIGH'
                        transfer = self.compute_cartesian_waypoints(
                            f'TRANSFER_CARTESIAN_HIGH_{label}',
                            [(place_tcp_x, place_tcp_y, self.safe_z)],
                            start_state=loaded_safe_corridor_state,
                            yaw=yaw,
                        )
                    except Exception as high_cartesian_error:
                        self.get_logger().warn(
                            f'{label}: high Cartesian transfer unavailable: '
                            f'{high_cartesian_error}; trying OMPL fallback'
                        )
                        transfer_mode = 'OMPL_FALLBACK'
                        transfer = self.plan_ompl_to(
                            f'TRANSFER_OMPL_{label}',
                            place_tcp_x,
                            place_tcp_y,
                            self.safe_z,
                            yaw=yaw,
                            start_state=loaded_safe_corridor_state,
                        )
                        transfer_joint_travel = (
                            self._trajectory_joint_travel(transfer)
                        )
                        if (
                            transfer_joint_travel
                            > self.MAX_OMPL_TRANSFER_JOINT_TRAVEL
                        ):
                            raise RuntimeError(
                                f'TRANSFER_OMPL_{label}: excessive joint '
                                f'travel {transfer_joint_travel:.3f} rad '
                                'exceeds '
                                f'{self.MAX_OMPL_TRANSFER_JOINT_TRAVEL:.3f} '
                                'rad'
                            )

                score = self._trajectory_joint_travel(
                    *approaches,
                    descent,
                    pick_exit,
                    *(() if transfer_lift is None else (transfer_lift,)),
                    transfer,
                )
                transfer_priority = {
                    'CARTESIAN_LOW': 0,
                    'CARTESIAN_HIGH': 1,
                    'OMPL_FALLBACK': 2,
                }[transfer_mode]
                feasible.append((
                    transfer_priority,
                    score,
                    abs(yaw),
                    yaw,
                    approaches,
                    descent,
                    pick_exit,
                    transfer_lift,
                    transfer,
                    transfer_mode,
                ))
                self.get_logger().info(
                    f'{label}: COMPLETE PREFLIGHT SUCCESS, '
                    f'transfer={transfer_mode}, '
                    f'joint_travel={score:.3f}rad'
                )
                direct_candidate_count = sum(
                    item[-1] == 'CARTESIAN_LOW' for item in feasible
                )
                if direct_candidate_count >= 3:
                    break
            except Exception as error:
                failures.append((yaw, str(error)))
                self.get_logger().warn(
                    f'{label}: candidate rejected: {error}'
                )

        if not feasible:
            diagnostics = []
            for yaw, error in failures[:3]:
                detail = self.diagnose_pose_goal(
                    x,
                    y,
                    pregrasp_z,
                    yaw=yaw,
                )
                diagnostics.append(
                    f'{math.degrees(yaw):+.0f}deg={detail}; planner={error}'
                )
            raise RuntimeError(
                'No complete grasp candidate; ' + ' | '.join(diagnostics)
            )

        # Any valid low Cartesian route wins over an OMPL fallback.  Within
        # the same transfer mode, retain the least-travelled yaw candidate.
        selected = min(
            feasible,
            key=lambda item: (item[0], item[1], item[2]),
        )
        (
            _transfer_priority,
            score,
            _abs_yaw,
            yaw,
            approaches,
            descent,
            pick_exit,
            transfer_lift,
            transfer,
            transfer_mode,
        ) = selected
        self.grasp_yaw = yaw
        self.get_logger().info(
            'SELECTED GRASP CANDIDATE: '
            f'yaw={math.degrees(yaw):+.1f}deg, '
            f'transfer={transfer_mode}, '
            f'joint_travel={score:.3f}rad, '
            f'feasible={len(feasible)}, '
            f'attempted={len(feasible) + len(failures)}'
        )
        return (
            approaches,
            descent,
            pick_exit,
            transfer_lift,
            transfer,
            transfer_mode,
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
            future,
            timeout_sec=5.0,
        )

        if not future.done():
            future.cancel()
            raise RuntimeError(
                'Timed out waiting for /suction_off response'
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

        self._attached = False
        if self._attached_collision:
            self._apply_attached_payload(False)

    def _set_phase(self, phase):
        self._phase = phase

    def _move_to_ready(self, name):
        """Move to the configured, collision-checked six-joint READY pose."""
        if self.ready_joint_values is None:
            raise RuntimeError(
                'AUBO_READY_JOINTS is required; automatic READY capture is '
                'disabled to keep the IK branch deterministic'
            )

        current = self.current_manipulator_joint_values()
        max_error = max(
            abs(
                (current[joint] - self.ready_joint_values[joint]
                 + math.pi) % (2.0 * math.pi) - math.pi
            )
            for joint in self.MANIPULATOR_JOINTS
        )
        tolerance = self.system_config.motion.ready_joint_tolerance
        if max_error <= tolerance:
            self.get_logger().info(
                f'{name}: already at READY; max joint error='
                f'{max_error:.4f} rad'
            )
            self.ended_at_ready = True
            return

        self.ompl_move_to_joint_values(
            name,
            self.ready_joint_values,
            tolerance=tolerance,
        )
        self.captured_ready_joint_values = dict(self.ready_joint_values)
        ordered = [
            self.ready_joint_values[joint]
            for joint in self.MANIPULATOR_JOINTS
        ]
        self.get_logger().info(
            'Using fixed READY joints (AUBO_READY_JOINTS): '
            + ','.join(f'{value:.6f}' for value in ordered)
        )

    def _classify_failure(self, error):
        detail = str(error).lower()
        if 'too far' in detail or (
            self._phase == 'SUCTION_ON'
            and 'suction' in detail
        ):
            return 'SUCTION_TOO_FAR'
        if (
            'outside safe' in detail
            or 'invalid pregrasp' in detail
        ):
            return 'VISION_INVALID'
        if 'collision' in detail or 'joint jump' in detail:
            return 'COLLISION_RISK'
        return 'PLANNING_FAILED'

    def recover_after_failure(self, x, y, pregrasp_z, contact_z):
        """Recover only from states whose robot pose is known exactly."""
        phase = self._phase
        deadline = time.monotonic() + self.recovery_timeout

        if x is None or y is None:
            self.get_logger().warn(
                'RECOVERY: no validated target pose; no motion attempted'
            )
            return

        if self._attached:
            if self._pick_descent_trajectory is None:
                raise RuntimeError(
                    f'payload held in {phase}; no verified reverse path exists'
                )

            # If transfer planning failed after the guided hopper exit, return
            # through the exact collision-checked path that was executed.
            # This restores the known pregrasp state before replacing the
            # object at its source position.
            if self._at_pick_corridor:
                if self._pick_exit_trajectory is None:
                    raise RuntimeError(
                        f'payload held in {phase}; no verified corridor '
                        'return path exists'
                    )
                self._at_pick_corridor = False
                self.execute_trajectory(
                    'RECOVERY_REVERSE_GUIDED_EXIT',
                    self.reverse_trajectory(self._pick_exit_trajectory),
                )
                self._at_pick_pregrasp = True

            # At contact, retreat by reversing the exact trajectory that was
            # executed during descent.  At pregrasp, use that same trajectory
            # forward to replace the object, then reverse it after release.
            if self._at_pick_contact:
                self._at_pick_contact = False
                self.execute_trajectory(
                    'RECOVERY_REVERSE_DESCENT',
                    self.reverse_trajectory(self._pick_descent_trajectory),
                )
                self._at_pick_pregrasp = True

            if self._at_pick_pregrasp:
                self._at_pick_pregrasp = False
                self.execute_trajectory(
                    'RECOVERY_REPLACE_AT_SOURCE',
                    self._pick_descent_trajectory,
                )
                self._at_pick_contact = True
                self.suction_off()
                self._at_pick_contact = False
                self.execute_trajectory(
                    'RECOVERY_RETREAT_FROM_SOURCE',
                    self.reverse_trajectory(self._pick_descent_trajectory),
                )
                self._at_pick_pregrasp = True

            if self._attached:
                raise RuntimeError(
                    f'payload held after failure in {phase}; robot pose is '
                    'not a verified source pose, so automatic release stopped'
                )

            # The object is back at its source and the robot is at the exact
            # pregrasp state.  Leave the hopper through the same verified
            # corridor and let the queue plan directly to the next target.
            # Returning to a fixed READY pose here is unnecessary and, for the
            # current workcell, can send wrist2/wrist3 through the hopper.
            if self._pick_exit_trajectory is None:
                self._pick_exit_trajectory = self.compute_cartesian_waypoints(
                    'RECOVERY_PLAN_EMPTY_EXIT',
                    [
                        (x, y, self.pick_clear_z),
                        (
                            self.exit_corridor_x,
                            self.exit_corridor_y,
                            self.pick_clear_z,
                        ),
                    ],
                    yaw=self.grasp_yaw,
                )

            if time.monotonic() >= deadline:
                raise RuntimeError('RECOVERY_CORRIDOR: recovery timeout')

            self._at_pick_pregrasp = False
            self.execute_trajectory(
                'RECOVERY_EXIT_EMPTY_TO_CORRIDOR',
                self._pick_exit_trajectory,
            )
            self._at_pick_corridor = True

            self._phase = 'RECOVERED_SOURCE_CORRIDOR'
            self.ended_at_ready = False
            self.get_logger().warn(
                'RECOVERY COMPLETE: payload returned to source; robot is at '
                'the verified exit corridor and READY was bypassed'
            )
            return

        # A suction service can attach the object before returning an error.
        # Only the uncertain SUCTION_ON phase needs a defensive suction_off;
        # failures that occurred before suction must not invoke it.
        if phase == 'SUCTION_ON':
            try:
                self.suction_off()
            except Exception as suction_error:
                self.get_logger().warn(
                    f'RECOVERY: defensive suction_off failed: {suction_error}'
                )

        no_motion_phases = (
            'INIT', 'TARGET_VALIDATED', 'READY_BOOTSTRAP',
            'PICK_PREFLIGHT',
        )
        if phase in no_motion_phases:
            self._phase = 'RECOVERY_NO_MOTION'
            self.get_logger().warn(
                'RECOVERY: validation failed before motion; no retreat needed'
            )
            return

        # A failed suction request is still at the verified contact pose after
        # the defensive suction_off above.  Retreat through the exact descent
        # trajectory, then leave through the standard empty corridor.  Other
        # execution failures have no verified pose and must stop instead of
        # attempting a blind move to READY.
        if self._at_pick_contact:
            self._at_pick_contact = False
            self.execute_trajectory(
                'RECOVERY_EMPTY_REVERSE_DESCENT',
                self.reverse_trajectory(self._pick_descent_trajectory),
            )
            self._at_pick_pregrasp = True

        if self._at_pick_pregrasp:
            if self._pick_exit_trajectory is None:
                self._pick_exit_trajectory = self.compute_cartesian_waypoints(
                    'RECOVERY_PLAN_EMPTY_EXIT',
                    [
                        (x, y, self.pick_clear_z),
                        (
                            self.exit_corridor_x,
                            self.exit_corridor_y,
                            self.pick_clear_z,
                        ),
                    ],
                    yaw=self.grasp_yaw,
                )
            if time.monotonic() >= deadline:
                raise RuntimeError('RECOVERY_CORRIDOR: recovery timeout')
            self._at_pick_pregrasp = False
            self.execute_trajectory(
                'RECOVERY_EXIT_EMPTY_TO_CORRIDOR',
                self._pick_exit_trajectory,
            )
            self._at_pick_corridor = True
            self._phase = 'RECOVERED_SOURCE_CORRIDOR'
            self.ended_at_ready = False
            self.get_logger().warn(
                'RECOVERY COMPLETE: robot is at the verified exit corridor; '
                'READY was bypassed'
            )
            return

        raise RuntimeError(
            f'cannot continue after failure in {phase}: robot pose is not '
            'verified and automatic READY recovery is disabled'
        )

    # =========================================================
    # Main pick-place sequence
    # =========================================================

    def run(self):

        x = None
        y = None
        object_x = None
        object_y = None
        object_top_z = None
        pregrasp_z = None
        contact_z = None

        try:

            # =================================================
            # 1. RGB-D target
            # =================================================

            if self.target_xyz is None:
                object_x, object_y, object_top_z = self.get_target()
            else:
                object_x, object_y, object_top_z = self.target_xyz
                self.get_logger().info(
                    'Using target selected by dynamic scene queue'
                )

            if self.grasp_xy is None:
                x, y = object_x, object_y
            else:
                x, y = self.grasp_xy

            payload_offset = (
                object_x - x,
                object_y - y,
            )
            place_tcp_x = self.place_x - payload_offset[0]
            place_tcp_y = self.place_y - payload_offset[1]

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'VISUAL DEEP-HOPPER PICK'
            )

            self.get_logger().info(
                f'object center = '
                f'({object_x:.3f}, {object_y:.3f}, {object_top_z:.3f})'
            )

            self.get_logger().info(
                f'grasp TCP = ({x:.3f}, {y:.3f}), '
                f'payload offset = ({payload_offset[0]:+.3f}, '
                f'{payload_offset[1]:+.3f})'
            )

            self.get_logger().info(
                '========================================'
            )

            # -------------------------------------------------
            # Basic target safety check
            #
            # Hopper bottom inner region is approximately:
            # x = hopper_x +/-0.3075
            # y = hopper_y +/-0.465
            #
            # The actual center limits below also subtract half the current
            # cube dimensions and AUBO_HOPPER_WALL_MARGIN.
            # -------------------------------------------------

            if not (
                self.hopper_x - self.safe_half_x
                < object_x < self.hopper_x + self.safe_half_x
                and self.hopper_y - self.safe_half_y
                < object_y < self.hopper_y + self.safe_half_y
                and
                -0.82 + self.hopper_dz < object_top_z < 0.05 + self.hopper_dz
            ):
                raise RuntimeError(
                    'Target is outside safe deep-hopper '
                    f'pick region: '
                    f'({object_x:.3f}, {object_y:.3f}, '
                    f'{object_top_z:.3f})'
                )

            self._set_phase('TARGET_VALIDATED')

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
            # 2. Use the current verified robot state
            # =================================================

            if self.bootstrap_ready:
                self.get_logger().info(
                    'STEP 0: establish canonical READY above hopper center'
                )
                self._set_phase('READY_BOOTSTRAP')
                self._move_to_ready('READY_BOOTSTRAP')
                self.ended_at_ready = True
            else:
                self.get_logger().info(
                    'STEP 0: use current verified joint state directly'
                )
                self.ended_at_ready = False

            # Preflight every phase that carries the payload before the arm
            # leaves its current state.  Each candidate uses a deliberate yaw
            # branch and includes a predicted attached cube during exit and
            # transfer collision checking.
            self.get_logger().info(
                'STEP 1: preflight APPROACH -> CONTACT -> LOADED EXIT '
                '-> LOW CARTESIAN TRANSFER '
                '(SAFE-LIFT + HIGH CARTESIAN + OMPL fallback)'
            )
            if self.staged_approach:
                self.get_logger().info(
                    'Adaptive retry: approach through hopper-centre stage'
                )

            self._set_phase('PICK_PREFLIGHT')
            (
                approach_trajectories,
                self._pick_descent_trajectory,
                self._pick_exit_trajectory,
                self._transfer_lift_trajectory,
                self._transfer_trajectory,
                self._transfer_mode,
            ) = self._preflight_pick_candidates(
                x,
                y,
                pregrasp_z,
                contact_z,
                payload_offset=payload_offset,
            )

            self.get_logger().info(
                'STEP 2: execute verified approach, then vertical descent'
            )
            self._set_phase('PICK_APPROACH')
            for index, trajectory in enumerate(approach_trajectories):
                name = (
                    'PICK_APPROACH_STAGE'
                    if len(approach_trajectories) > 1 and index == 0
                    else 'PICK_APPROACH'
                )
                self.execute_trajectory(name, trajectory)
            self.ended_at_ready = False
            self._at_pick_pregrasp = True

            self._set_phase('PICK_DESCEND')
            self._at_pick_pregrasp = False
            self.execute_trajectory(
                'PICK_DESCEND',
                self._pick_descent_trajectory,
            )
            self._at_pick_contact = True

            time.sleep(0.3)

            # =================================================
            # 6. Suction ON
            # =================================================

            self.get_logger().info(
                'STEP 3: SUCTION ON'
            )

            self._set_phase('SUCTION_ON')
            self.suction_on()
            self._attached = True
            self._apply_attached_payload(
                True,
                (object_x, object_y, object_top_z),
            )

            time.sleep(0.4)

            # =================================================
            # 7. First lift inside hopper
            # =================================================

            self.get_logger().info(
                'STEP 4: reverse verified descent, then exit hopper'
            )

            self._set_phase('PICK_LIFT')
            self._at_pick_contact = False
            self.execute_trajectory(
                'PICK_LIFT',
                self.reverse_trajectory(self._pick_descent_trajectory),
            )
            self._at_pick_pregrasp = True

            # The loaded exit was already checked from the exact future
            # pregrasp state before any robot motion.  Execute that verified
            # path rather than planning for the first time after suction.
            self._set_phase('PICK_EXIT_GUIDED')
            # Planning failure leaves the robot at the verified pregrasp and
            # permits source replacement.  Once execution starts, clear that
            # flag because a controller error could leave an intermediate,
            # unverified pose.
            self._at_pick_pregrasp = False
            self.execute_trajectory(
                'PICK_EXIT_GUIDED',
                self._pick_exit_trajectory,
            )
            self._at_pick_corridor = True
            time.sleep(0.3)

            # =================================================
            # 10. Transfer to receiving bin
            # =================================================

            self.get_logger().info(
                f'STEP 5: {self._transfer_mode} transfer to PLACE ENTRY'
            )

            self._set_phase('TRANSFER_GUIDED')
            # The transfer was preflighted before approach.  Once execution
            # starts, clear the corridor flag because a controller failure may
            # leave an intermediate pose that must not be treated as verified.
            self._at_pick_corridor = False
            if self._transfer_lift_trajectory is not None:
                self.execute_trajectory(
                    'TRANSFER_LIFT_SAFE',
                    self._transfer_lift_trajectory,
                )
            self.execute_trajectory(
                self._transfer_mode,
                self._transfer_trajectory,
            )

            self._set_phase('TRANSFER_DONE')

            time.sleep(0.3)

            # =================================================
            # 11. Enter receiving bin
            # =================================================

            self.get_logger().info(
                'STEP 6: Cartesian vertical PLACE ENTRY'
            )

            self._set_phase('PLACE_ENTRY')
            self.cartesian_move_to(
                'PLACE_ENTRY',
                place_tcp_x,
                place_tcp_y,
                self.place_entry_z,
                yaw=self.grasp_yaw,
            )

            time.sleep(0.2)

            # =================================================
            # 12. Final place descent
            # =================================================

            self.get_logger().info(
                'STEP 7: Cartesian vertical PLACE FINAL DESCEND'
            )

            self._set_phase('PLACE_DESCEND')
            self.cartesian_move_to(
                'PLACE_DESCEND',
                place_tcp_x,
                place_tcp_y,
                self.place_release_z,
                yaw=self.grasp_yaw,
            )

            time.sleep(0.3)

            # =================================================
            # 13. Release
            # =================================================

            self.get_logger().info(
                'STEP 8: SUCTION OFF'
            )

            self._set_phase('SUCTION_OFF')
            self.suction_off()

            time.sleep(0.20)

            # =================================================
            # 14. Retreat inside receiving bin
            # =================================================

            self.get_logger().info(
                'STEP 9: Cartesian vertical PLACE RETREAT INNER'
            )

            self._set_phase('PLACE_RETREAT_INNER')
            self.cartesian_move_to(
                'PLACE_RETREAT_INNER',
                place_tcp_x,
                place_tcp_y,
                self.place_entry_z,
                yaw=self.grasp_yaw,
            )

            time.sleep(0.2)

            # =================================================
            # 15. Return to global safe height
            # =================================================

            self.get_logger().info(
                'STEP 10: Cartesian vertical PLACE RETREAT SAFE'
            )

            self._set_phase('PLACE_RETREAT_SAFE')
            self.cartesian_move_to(
                'PLACE_RETREAT_SAFE',
                place_tcp_x,
                place_tcp_y,
                self.safe_z,
                yaw=self.grasp_yaw,
            )

            time.sleep(0.3)

            # A successful cycle remains at PLACE SAFE.  The next target is
            # planned from this real state. READY is reserved for recovery and
            # one final park after the complete queue finishes.
            self.ended_at_ready = False

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

            failure = e if isinstance(e, PickFailure) else PickFailure(
                self._classify_failure(e),
                self._phase,
                e,
            )

            self.get_logger().error(
                '========================================'
            )

            self.get_logger().error(
                f'VISUAL PICK FAILED: {failure}'
            )

            self.get_logger().error(
                '========================================'
            )

            try:
                self.recover_after_failure(
                    x,
                    y,
                    pregrasp_z,
                    contact_z,
                )
            except Exception as recovery_error:
                self.get_logger().error(
                    f'RECOVERY FAILED: {recovery_error}'
                )
                raise PickFailure(
                    'RECOVERY_FAILED',
                    self._phase,
                    recovery_error,
                ) from recovery_error

            if failure is e:
                raise
            raise failure from e


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

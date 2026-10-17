#!/usr/bin/env python3

"""
Dynamic visual pick scheduler.

This replaces the fixed `target_count == 4` assumption with a scene queue.
The detector publishes all current target poses on `/vision/target_array`.
After each pick/place attempt the scheduler waits for a new stable snapshot,
matches targets by 3-D position, adds newly exposed targets, and only removes
the reserved target after the post-action scene confirms it disappeared.
"""

import copy
import time
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32

try:
    from .system_config import load_system_config
    from .target_queue import TargetQueue
    from .visual_pick_to_bin import PickFailure, VisualPickToBin
except ImportError:
    # Keep direct execution from the source directory working for the shell
    # scripts and developer debugging.
    from system_config import load_system_config
    from target_queue import TargetQueue
    from visual_pick_to_bin import PickFailure, VisualPickToBin


class TargetArrayMonitor(Node):

    MANIPULATOR_JOINTS = (
        'shoulder_joint',
        'upperArm_joint',
        'foreArm_joint',
        'wrist1_joint',
        'wrist2_joint',
        'wrist3_joint',
    )

    def __init__(self, config):
        super().__init__('target_array_monitor')
        self.latest_positions = None
        self.last_update = None
        self.update_sequence = 0
        self.source_evidence_count = None
        self.source_evidence_update = None
        self.latest_ambiguous_positions = []
        self.ambiguous_update = None
        self.create_subscription(
            PoseArray,
            '/vision/target_array',
            self.target_array_cb,
            10,
        )
        self.create_subscription(
            Int32,
            '/vision/source_evidence_count',
            self.source_evidence_cb,
            10,
        )
        self.create_subscription(
            PoseArray,
            '/vision/ambiguous_regions',
            self.ambiguous_regions_cb,
            10,
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )
        self.dynamic_scene_ids = set()
        self.latest_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            20,
        )
        self.state_validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
        )
        workcell = config.workcell
        self.cube_size = (
            workcell.cube_x,
            workcell.cube_y,
            workcell.cube_z,
        )
        self.collision_padding = config.queue.collision_padding
        self.stable_frames = max(2, config.queue.stable_frames)
        self.ambiguous_obstacle_scale = max(
            1.0, config.queue.ambiguous_obstacle_scale
        )

    def joint_state_cb(self, msg):
        self.latest_joint_state = copy.deepcopy(msg)

    def wait_for_stable_robot_state(
        self,
        timeout=5.0,
        stable_time=0.30,
        tolerance=0.003,
    ):
        """Wait for finite, complete and stationary manipulator joints."""
        start = time.monotonic()
        stable_since = None
        previous = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - start > timeout:
                raise RuntimeError(
                    'Robot joint state was not complete and stable within '
                    f'{timeout:.1f}s'
                )
            if self.latest_joint_state is None:
                continue

            positions = dict(zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            ))
            if any(name not in positions for name in self.MANIPULATOR_JOINTS):
                continue
            current = tuple(
                float(positions[name]) for name in self.MANIPULATOR_JOINTS
            )
            if not all(math.isfinite(value) for value in current):
                stable_since = None
                previous = None
                continue

            if previous is None:
                stable_since = now
            else:
                max_delta = max(
                    abs((right - left + math.pi) % (2.0 * math.pi) - math.pi)
                    for left, right in zip(previous, current)
                )
                if max_delta > tolerance:
                    stable_since = now
            previous = current

            if stable_since is not None and now - stable_since >= stable_time:
                return current

        raise RuntimeError('ROS shutdown while waiting for robot state')

    def validate_current_robot_state(self):
        """Reject startup when MoveIt considers the current state invalid."""
        if self.latest_joint_state is None:
            raise RuntimeError('No current joint state is available')
        if not self.state_validity_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/check_state_validity is unavailable')

        request = GetStateValidity.Request()
        request.group_name = 'manipulator'
        request.robot_state = RobotState()
        request.robot_state.joint_state = copy.deepcopy(
            self.latest_joint_state
        )
        request.robot_state.is_diff = False
        future = self.state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            raise RuntimeError('No /check_state_validity response')
        if not response.valid:
            contacts = [
                f'{contact.contact_body_1}<->{contact.contact_body_2}'
                for contact in response.contacts[:5]
            ]
            detail = ', '.join(contacts) if contacts else 'unknown collision'
            raise RuntimeError(
                f'Current Gazebo robot state is invalid: {detail}'
            )

    def target_array_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != 'base_link':
            return
        self.latest_positions = [
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for pose in msg.poses
        ]
        self.last_update = time.monotonic()
        self.update_sequence += 1

    def source_evidence_cb(self, msg):
        self.source_evidence_count = int(msg.data)
        self.source_evidence_update = time.monotonic()

    def ambiguous_regions_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != 'base_link':
            return
        self.latest_ambiguous_positions = sorted([
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for pose in msg.poses
        ])
        self.ambiguous_update = time.monotonic()

    def filter_ambiguous_targets(self, positions):
        """Never queue a point that lies inside an unresolved red region."""
        half_x = self.cube_size[0] * self.ambiguous_obstacle_scale * 0.5
        half_y = self.cube_size[1] * self.ambiguous_obstacle_scale * 0.5
        return [
            point for point in positions
            if not any(
                abs(point[0] - region[0]) <= half_x
                and abs(point[1] - region[1]) <= half_y
                for region in self.latest_ambiguous_positions
            )
        ]

    @staticmethod
    def _max_position_delta(left, right):
        if len(left) != len(right):
            return math.inf

        return max(
            math.dist(first, second)
            for first, second in zip(left, right)
        ) if left else 0.0

    def wait_for_stable_snapshot(
        self,
        stable_time,
        timeout=20.0,
        min_sequence=0,
        message_timeout=2.50,
        position_tolerance=0.015,
    ):
        """
        Wait for a fresh, position-stable target-array snapshot.

        A stable count alone is unsafe: if the detector dies, the last array
        would otherwise look permanently stable.  The caller can also pass a
        sequence captured before an action so that the post-action snapshot
        must contain at least one newer detector message.
        """
        start = time.monotonic()
        stable_since = None
        stable_frame_count = 0
        previous_positions = None
        previous_ambiguous = None
        processed_sequence = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.10)
            now = time.monotonic()

            if now - start > timeout:
                if self.latest_positions is None:
                    raise RuntimeError(
                        'No /vision/target_array messages received; '
                        'detector pipeline is not publishing'
                    )
                age = (
                    'unknown'
                    if self.last_update is None
                    else f'{now - self.last_update:.2f}s'
                )
                if (
                    not self.latest_positions
                    and self.source_evidence_count is not None
                    and self.source_evidence_count > 0
                ):
                    raise RuntimeError(
                        'Red contours remain inside the source hopper but all '
                        'candidates were rejected; inspect VISION FRAME '
                        'reject counters instead of treating the scene as '
                        'empty'
                    )
                raise RuntimeError(
                    'Target-array messages were received but never became '
                    f'stable; updates={self.update_sequence}, '
                    f'last_count={len(self.latest_positions)}, age={age}'
                )

            if self.latest_positions is None:
                continue

            if self.update_sequence <= min_sequence:
                continue

            if (
                self.last_update is None
                or now - self.last_update > message_timeout
            ):
                age = (
                    'unknown'
                    if self.last_update is None
                    else f'{now - self.last_update:.2f}'
                )
                raise RuntimeError(
                    'Vision target_array stream is stale; '
                    f'last update age={age}s'
                )

            if processed_sequence == self.update_sequence:
                continue

            current_positions = self.filter_ambiguous_targets(
                list(self.latest_positions)
            )
            current_ambiguous = list(self.latest_ambiguous_positions)
            processed_sequence = self.update_sequence

            # An empty target array is a valid "scene empty" observation only
            # when the detector also confirms no red evidence in the source
            # hopper.  Red objects already placed in the receiving bin do not
            # increment source_evidence_count.
            evidence_fresh = (
                self.source_evidence_update is not None
                and now - self.source_evidence_update <= message_timeout
            )
            if not current_positions and (
                not evidence_fresh
                or self.source_evidence_count is None
                or self.source_evidence_count > 0
            ):
                previous_positions = None
                previous_ambiguous = None
                stable_since = None
                stable_frame_count = 0
                continue

            if (
                previous_positions is None
                or self._max_position_delta(
                    previous_positions,
                    current_positions,
                ) > position_tolerance
                or previous_ambiguous is None
                # These regions become collision obstacles. A moving region
                # must settle even when its count stays unchanged.
                or self._max_position_delta(
                    previous_ambiguous,
                    current_ambiguous,
                ) > position_tolerance
            ):
                previous_positions = current_positions
                previous_ambiguous = current_ambiguous
                stable_since = now
                stable_frame_count = 1
                continue

            if stable_since is None:
                stable_since = now
                stable_frame_count = 1
                continue

            stable_frame_count += 1

            if (
                now - stable_since >= stable_time
                and stable_frame_count >= self.stable_frames
            ):
                return current_positions

    def apply_target_obstacles(self, records, selected_id=None):
        """Mirror visible non-selected cubes into the MoveIt world."""
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/apply_planning_scene unavailable')

        visible_records = [
            record for record in records
            if record.target_id != selected_id
            and record.state not in ('MISSING', 'WAITING_CONFIRMATION')
        ]
        wanted_ids = {
            f'vision_cube_{record.target_id}'
            for record in visible_records
        }
        wanted_ids.update(
            f'vision_ambiguous_{index}'
            for index, _point in enumerate(self.latest_ambiguous_positions)
        )
        objects = []

        for stale_id in sorted(self.dynamic_scene_ids - wanted_ids):
            stale = CollisionObject()
            stale.header.frame_id = 'base_link'
            stale.id = stale_id
            stale.operation = CollisionObject.REMOVE
            objects.append(stale)

        for record in visible_records:
            obstacle = CollisionObject()
            obstacle.header.frame_id = 'base_link'
            obstacle.id = f'vision_cube_{record.target_id}'
            obstacle.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                self.cube_size[0] + self.collision_padding * 2.0,
                self.cube_size[1] + self.collision_padding * 2.0,
                self.cube_size[2] + self.collision_padding * 2.0,
            ]

            pose = Pose()
            pose.position.x = record.position[0]
            pose.position.y = record.position[1]
            # Vision reports the top surface.  Convert it to box center and
            # keep half the padding above/below the physical cube.
            pose.position.z = record.position[2] - self.cube_size[2] * 0.5
            pose.orientation.w = 1.0
            obstacle.primitives.append(primitive)
            obstacle.primitive_poses.append(pose)
            objects.append(obstacle)

        for index, point in enumerate(self.latest_ambiguous_positions):
            obstacle = CollisionObject()
            obstacle.header.frame_id = 'base_link'
            obstacle.id = f'vision_ambiguous_{index}'
            obstacle.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                self.cube_size[0] * self.ambiguous_obstacle_scale,
                self.cube_size[1] * self.ambiguous_obstacle_scale,
                self.cube_size[2] + self.collision_padding * 2.0,
            ]
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2] - self.cube_size[2] * 0.5
            pose.orientation.w = 1.0
            obstacle.primitives.append(primitive)
            obstacle.primitive_poses.append(pose)
            objects.append(obstacle)

        if not objects and wanted_ids == self.dynamic_scene_ids:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                'Failed to update visual cube collision objects'
            )
        self.dynamic_scene_ids = wanted_ids

    def clear_target_obstacles(self):
        if not self.dynamic_scene_ids:
            return
        saved_ambiguous = self.latest_ambiguous_positions
        try:
            self.latest_ambiguous_positions = []
            self.apply_target_obstacles([], selected_id=None)
        except Exception as error:
            self.get_logger().warn(
                f'Could not clear visual collision objects: {error}'
            )
        finally:
            self.latest_ambiguous_positions = saved_ambiguous


def scene_summary(queue):
    counts = {}
    for record in queue.records:
        counts[record.state] = counts.get(record.state, 0) + 1
    return ', '.join(
        f'{state}={count}' for state, count in sorted(counts.items())
    ) or 'empty'


def receiving_bin_place_slots(workcell):
    """Return six non-overlapping cube centres inside the receiving bin."""
    # The current SDF store has an approximately 0.35 x 0.50 m inner opening
    # in base_link after its -90-degree rotation.  A two-by-three grid leaves
    # 10 mm between X footprints, at least 35 mm between Y footprints, and
    # about 20 mm or more to every wall for a 0.15 x 0.10 m cube.
    offsets = (
        (-0.655, -0.740),
        (-0.495, -0.740),
        (-0.655, -0.895),
        (-0.495, -0.895),
        (-0.655, -1.030),
        (-0.495, -1.030),
    )
    slots = tuple(
        (workcell.base_x + x, workcell.base_y + y)
        for x, y in offsets
    )

    inner_x = (workcell.base_x - 0.750, workcell.base_x - 0.400)
    inner_y = (workcell.base_y - 1.146, workcell.base_y - 0.646)
    wall_margin = 0.005
    for index, (x, y) in enumerate(slots, start=1):
        if not (
            x - workcell.cube_x * 0.5 >= inner_x[0] + wall_margin
            and x + workcell.cube_x * 0.5 <= inner_x[1] - wall_margin
            and y - workcell.cube_y * 0.5 >= inner_y[0] + wall_margin
            and y + workcell.cube_y * 0.5 <= inner_y[1] - wall_margin
        ):
            raise RuntimeError(
                f'Receiving-bin slot {index} is outside the safe interior'
            )

    for left_index, left in enumerate(slots):
        for right in slots[left_index + 1:]:
            separated_x = (
                abs(left[0] - right[0])
                >= workcell.cube_x + wall_margin
            )
            separated_y = (
                abs(left[1] - right[1])
                >= workcell.cube_y + wall_margin
            )
            if not (separated_x or separated_y):
                raise RuntimeError(
                    'Receiving-bin place slots overlap for the configured '
                    'cube dimensions'
                )
    return slots


def validate_system_start(monitor):
    """Validate the actual Gazebo state without moving to READY."""
    joints = monitor.wait_for_stable_robot_state()
    monitor.validate_current_robot_state()
    print(
        '[SYSTEM START] current joints stable and collision-free: '
        + ','.join(f'{value:.4f}' for value in joints),
        flush=True,
    )


def park_at_ready(config):
    """Park once after normal queue completion."""
    if config.motion.ready_joint_values is None:
        raise RuntimeError(
            'AUBO_READY_JOINTS is empty; final parking is unavailable'
        )
    parker = VisualPickToBin(
        node_name='aubo_final_ready_parker',
        bootstrap_ready=False,
        ready_joint_values=config.motion.ready_joint_values,
        use_global_arguments=False,
    )
    try:
        parker._set_phase('FINAL_PARK_READY')
        parker._move_to_ready('FINAL_PARK_READY')
        print('[FINAL PARK] fixed six-joint READY reached.', flush=True)
    finally:
        parker.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    config = load_system_config()
    workcell = config.workcell
    queue_config = config.queue
    monitor = TargetArrayMonitor(config)

    queue = TargetQueue(
        match_distance=queue_config.match_distance,
        missing_confirmations=queue_config.missing_confirmations,
        max_retries=queue_config.max_retries,
        edge_margin=queue_config.edge_margin,
        edge_max_retries=queue_config.edge_max_retries,
        hopper_center=(workcell.hopper_x, workcell.hopper_y),
        safe_half=(workcell.safe_half_x, workcell.safe_half_y),
    )

    stable_time = queue_config.stable_time
    empty_stable_time = queue_config.empty_stable_time
    message_timeout = queue_config.message_timeout
    position_tolerance = queue_config.stability_distance
    snapshot_timeout = queue_config.snapshot_timeout
    settle_time = queue_config.settle_time
    picked = 0
    pick_cycle = 0
    ready_joint_values = config.motion.ready_joint_values

    # Six unique positions prevent later cubes from being dropped onto an
    # earlier placement.  Geometry is validated once before robot motion.
    place_slots = receiving_bin_place_slots(workcell)

    print()
    print('=' * 68)
    print('DYNAMIC RGB-D TARGET QUEUE PICK')
    print('Detect all -> queue -> pick -> place -> re-detect -> reconcile')
    print(
        'Edge grasp compensation: '
        f'enabled={queue_config.edge_grasp_compensation}, '
        f'trigger={queue_config.edge_grasp_trigger:.3f} m, '
        f'offset={queue_config.edge_grasp_offset:.3f} m'
    )
    print('=' * 68)

    try:
        # Startup is observational: use Gazebo's actual settled state and do
        # not insert an otherwise unnecessary move through the camera holder.
        validate_system_start(monitor)

        initial_positions = monitor.wait_for_stable_snapshot(
            stable_time=stable_time,
            timeout=snapshot_timeout,
            message_timeout=message_timeout,
            position_tolerance=position_tolerance,
        )
        queue.seed(initial_positions)

        print(
            f'[SCENE INIT] detected={len(initial_positions)} | '
            f'queue={scene_summary(queue)}',
            flush=True,
        )

        if not initial_positions:
            print('No visible targets. Dynamic pick completed.', flush=True)

        run_queue = bool(initial_positions)
        while rclpy.ok() and run_queue:
            target = queue.reserve_next()

            if target is None:
                if queue.reactivate_deferred():
                    print(
                        '[QUEUE] deferred planning failures released after '
                        'other targets were processed.',
                        flush=True,
                    )
                    continue

                if (
                    queue.records
                    and not queue.pending()
                    and not any(
                        record.state == 'MISSING'
                        for record in queue.records
                    )
                ):
                    print(
                        '[STOP] no retryable targets remain; '
                        'failed targets stay recorded.',
                        flush=True,
                    )
                    break

                # There may be temporarily missing records.  Re-detect before
                # deciding that the source scene is empty.
                if not queue.unresolved():
                    scene_start_sequence = monitor.update_sequence
                    positions = monitor.wait_for_stable_snapshot(
                        stable_time=empty_stable_time,
                        timeout=15.0,
                        min_sequence=scene_start_sequence,
                        message_timeout=message_timeout,
                        position_tolerance=position_tolerance,
                    )
                    if not positions:
                        print(
                            '[SCENE EMPTY] no visible targets remain.',
                            flush=True,
                        )
                        break
                    queue.reconcile(positions)
                    continue

                scene_start_sequence = monitor.update_sequence
                positions = monitor.wait_for_stable_snapshot(
                    stable_time=stable_time,
                    timeout=15.0,
                    min_sequence=scene_start_sequence,
                    message_timeout=message_timeout,
                    position_tolerance=position_tolerance,
                )
                added, removed = queue.reconcile(positions)
                print(
                    f'[SCENE UPDATE] detected={len(positions)} | '
                    f'added={len(added)} | removed={len(removed)} | '
                    f'queue={scene_summary(queue)}',
                    flush=True,
                )
                continue

            print()
            pick_cycle += 1
            print(
                f'[PICK cycle={pick_cycle} confirmed={picked}] '
                f'id={target.target_id} '
                f'position=({target.position[0]:.3f}, '
                f'{target.position[1]:.3f}, '
                f'{target.position[2]:.3f}) '
                f'distance={target.distance:.3f} '
                f'clearance={queue.nearest_neighbor_distance(target):.3f} '
                f'edge={queue.is_edge_target(target)} '
                f'attempt={target.attempts}',
                flush=True,
            )

            # The selected target is intentionally omitted so the TCP can
            # contact it.  Every other visible cube remains a collision body.
            # Freeze this target for the complete pick.  Detector callbacks
            # may continue for debug output, but queue reconciliation happens
            # only after robot motion has finished and a fresh stable snapshot
            # is available.
            target_position = tuple(target.position)
            staged_approach = (
                queue.is_edge_target(target)
                and target.attempts > 1
            )
            grasp_position, grasp_reason = queue.compensated_grasp_position(
                target,
                enabled=queue_config.edge_grasp_compensation,
                trigger=queue_config.edge_grasp_trigger,
                offset=queue_config.edge_grasp_offset,
            )
            print(
                '[GRASP POINT] '
                f'object=({target_position[0]:.3f}, '
                f'{target_position[1]:.3f}) -> '
                f'tcp=({grasp_position[0]:.3f}, '
                f'{grasp_position[1]:.3f}) '
                f'reason={grasp_reason} '
                f'staged_approach={staged_approach}',
                flush=True,
            )
            monitor.apply_target_obstacles(
                queue.records,
                selected_id=target.target_id,
            )

            picker = VisualPickToBin(
                target_xyz=target_position,
                grasp_xy=grasp_position[:2],
                staged_approach=staged_approach,
                node_name=(
                    f'auto_suction_grasp_{target.target_id}_'
                    f'{target.attempts}'
                ),
                bootstrap_ready=False,
                ready_joint_values=ready_joint_values,
                use_global_arguments=False,
            )
            if picked >= len(place_slots):
                raise RuntimeError(
                    'Receiving bin has no unused placement slot remaining'
                )
            picker.place_x, picker.place_y = place_slots[picked]
            print(
                f'[PLACE SLOT] {picked + 1}/{len(place_slots)} '
                f'({picker.place_x:.3f}, {picker.place_y:.3f})',
                flush=True,
            )

            success = False
            failure_reason = 'PLANNING_FAILED'
            fatal_failure = None
            try:
                picker.run()
                success = True
            except PickFailure as error:
                failure_reason = error.reason
                if error.reason == 'RECOVERY_FAILED':
                    fatal_failure = str(error)
                print(
                    f'[PICK FAILED] id={target.target_id} '
                    f'reason={failure_reason}: {error}',
                    flush=True,
                )
            except Exception as error:
                print(
                    f'[PICK FAILED] id={target.target_id}: {error}',
                    flush=True,
                )
            finally:
                picker.destroy_node()

            if fatal_failure is not None:
                queue.hold_for_confirmation(target.target_id)
                raise RuntimeError(
                    'Safety stop after recovery failure; queue will not '
                    f'continue with stale robot/payload state: {fatal_failure}'
                )

            # Let Gazebo settle before comparing the post-action scene.
            time.sleep(settle_time)
            # Frames received during robot motion or settling are forbidden
            # as post-action evidence.  Require a new synchronized RGB-D
            # publication after this exact point.
            scene_start_sequence = monitor.update_sequence
            post_positions = None
            try:
                post_positions = monitor.wait_for_stable_snapshot(
                    stable_time=stable_time,
                    timeout=snapshot_timeout,
                    min_sequence=scene_start_sequence,
                    message_timeout=message_timeout,
                    position_tolerance=position_tolerance,
                )
            except Exception as error:
                # Do not restart or replace a detector behind the queue. The
                # unified launch owns both nodes. Preserve the reserved target
                # and stop because old coordinates are unsafe after motion.
                queue.hold_for_confirmation(target.target_id)
                print(
                    f'[STOP] post-action scene unavailable for '
                    f'id={target.target_id}; queue paused in '
                    f'WAITING_CONFIRMATION: {error}',
                    flush=True,
                )
                raise RuntimeError(
                    'Cannot continue without a fresh post-action visual scene'
                ) from error

            target_removed = queue.finish_attempt(
                target.target_id,
                post_positions,
                success,
                failure_reason=failure_reason,
            )
            if target_removed:
                picked += 1

            print(
                f'[SCENE UPDATE] detected={len(post_positions)} | '
                f'target_removed={target_removed} | '
                f'queue={scene_summary(queue)}',
                flush=True,
            )

        # Preserve every remaining source cube as a collision object while
        # executing the one intentional final move to the parking posture.
        monitor.apply_target_obstacles(queue.records, selected_id=None)
        park_at_ready(config)

    except KeyboardInterrupt:
        print('\nInterrupted.', flush=True)
        return 130
    except Exception as error:
        print(f'DYNAMIC PICK FAILED: {error}', flush=True)
        return 1
    finally:
        monitor.clear_target_obstacles()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    terminal_failures = [
        record for record in queue.records
        if record.state in ('BLOCKED', 'FAILED')
    ]
    if terminal_failures:
        details = ', '.join(
            f'id={record.target_id}:{record.state}:{record.failure_reason}'
            for record in terminal_failures
        )
        print(
            'DYNAMIC PICK FAILED: source targets remain unplaced: '
            + details,
            flush=True,
        )
        return 1

    print()
    print('=' * 68)
    print(f'DYNAMIC PICK FINISHED | confirmed placed={picked}')
    print('=' * 68)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

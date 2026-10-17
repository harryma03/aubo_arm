#!/usr/bin/env python3

"""Scene target records and matching for dynamic visual picking."""

from dataclasses import dataclass, field
import math
import time


@dataclass
class TargetRecord:
    target_id: int
    position: tuple
    state: str = 'PENDING'
    miss_count: int = 0
    attempts: int = 0
    failure_reason: str = ''
    last_seen: float = field(default_factory=time.monotonic)

    @property
    def distance(self):
        # All positions are in base_link.  This is a stable geometric proxy
        # for robot-base distance and does not depend on a stale target pose.
        return math.sqrt(sum(value * value for value in self.position))


class TargetQueue:
    """
    Priority queue with one-to-one scene association.

    The queue intentionally does not use the current target count as identity.
    Identical objects are associated by nearest 3-D position with a gate.
    Missing records are held temporarily to tolerate occlusion and settling.
    """

    def __init__(
        self,
        match_distance=0.120,
        missing_confirmations=3,
        max_retries=1,
        edge_margin=0.0,
        edge_max_retries=0,
        hopper_center=(0.0, 0.0),
        safe_half=(float('inf'), float('inf')),
    ):
        self.match_distance = float(match_distance)
        self.missing_confirmations = int(missing_confirmations)
        self.max_retries = int(max_retries)
        self.edge_margin = max(0.0, float(edge_margin))
        self.edge_max_retries = max(0, int(edge_max_retries))
        self.hopper_center = tuple(float(value) for value in hopper_center)
        self.safe_half = tuple(float(value) for value in safe_half)
        self.records = []
        self.next_id = 1

    @staticmethod
    def _distance(left, right):
        return math.sqrt(sum(
            (left[index] - right[index]) ** 2
            for index in range(3)
        ))

    @staticmethod
    def _point(pose):
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )

    def seed(self, positions):
        self.records.clear()
        self.next_id = 1
        for position in positions:
            self._append(position)
        self._sort()

    def _append(self, position):
        record = TargetRecord(
            target_id=self.next_id,
            position=tuple(position),
        )
        self.next_id += 1
        self.records.append(record)
        return record

    def _sort(self):
        self.records.sort(key=lambda record: record.distance)

    def pending(self):
        return [
            record for record in self.records
            if record.state == 'PENDING'
        ]

    def nearest_neighbor_distance(self, target):
        """Return clearance to the nearest still-physical target record."""
        neighbours = [
            record for record in self.records
            if record.target_id != target.target_id
            and record.state not in ('MISSING', 'WAITING_CONFIRMATION')
        ]
        if not neighbours:
            return float('inf')
        return min(
            self._distance(target.position, record.position)
            for record in neighbours
        )

    def is_edge_target(self, record):
        """Return whether a target is close to the configured safe ROI edge."""
        if self.edge_margin <= 0.0:
            return False

        remaining_x = (
            self.safe_half[0]
            - abs(record.position[0] - self.hopper_center[0])
        )
        remaining_y = (
            self.safe_half[1]
            - abs(record.position[1] - self.hopper_center[1])
        )
        return min(remaining_x, remaining_y) <= self.edge_margin

    def compensated_grasp_position(
        self,
        target,
        enabled=True,
        trigger=0.080,
        offset=0.020,
    ):
        """Return a graded wall-aware TCP point without changing identity."""
        original = tuple(float(value) for value in target.position)
        if not enabled or trigger <= 0.0 or offset <= 0.0:
            return original, 'CENTER_ONLY'

        x, y, z = original
        center_x, center_y = self.hopper_center
        remaining_x = self.safe_half[0] - abs(x - center_x)
        remaining_y = self.safe_half[1] - abs(y - center_y)

        grasp_x = x
        grasp_y = y
        axes = []

        def axis_offset(clearance):
            # Base 20 mm compensation is increased in 5 mm steps as the
            # target approaches a wall.  A second reserved attempt promotes
            # the axis by one level, while retaining the 30 mm upper bound.
            level = 0
            if clearance <= 0.025:
                level = 2
            elif clearance <= 0.050:
                level = 1
            if target.attempts >= 2:
                level = min(level + 1, 2)
            return float(offset) + level * 0.005

        if remaining_x <= trigger and x != center_x:
            shift = min(axis_offset(remaining_x), abs(x - center_x))
            grasp_x -= math.copysign(shift, x - center_x)
            axes.append('X')
        if remaining_y <= trigger and y != center_y:
            shift = min(axis_offset(remaining_y), abs(y - center_y))
            grasp_y -= math.copysign(shift, y - center_y)
            axes.append('Y')

        if not axes:
            return original, 'CENTER_NOT_NEAR_WALL'

        candidate = (grasp_x, grasp_y, z)
        target_distance = self._distance(candidate, original)
        nearest_margin = 0.010
        neighbours = [
            record for record in self.records
            if record.target_id != target.target_id
            and record.state != 'MISSING'
        ]
        if neighbours:
            nearest_other = min(
                self._distance(candidate, record.position)
                for record in neighbours
            )
            if target_distance + nearest_margin >= nearest_other:
                return original, 'CENTER_NEAREST_TARGET_GUARD'

        return candidate, 'INWARD_' + ''.join(axes)

    def unresolved(self):
        return [
            record for record in self.records
            if record.state in (
                'PENDING',
                'MISSING',
                'RESERVED',
                'WAITING_CONFIRMATION',
                'DEFERRED',
            )
        ]

    def reserve_next(self):
        candidates = self.pending()
        if not candidates:
            return None
        # Prefer targets that are less likely to be blocked by the hopper or
        # by the surrounding pile.  Height is evaluated before clearance so a
        # cube sitting on top of another cube is removed first.  Robot-base
        # distance is only a final tie-breaker; the nearest cube is not always
        # the easiest or safest cube to approach.
        target = min(
            candidates,
            key=lambda record: (
                self.is_edge_target(record),
                -record.position[2],
                -self.nearest_neighbor_distance(record),
                record.distance,
            ),
        )
        target.state = 'RESERVED'
        target.attempts += 1
        return target

    def hold_for_confirmation(self, target_id):
        """Keep a reserved target locked when no fresh scene is available."""
        target = self._find(target_id)
        if target is not None:
            target.state = 'WAITING_CONFIRMATION'

    def fail(self, target_id, reason='PLANNING_FAILED'):
        target = self._find(target_id)
        if target is None:
            return

        target.failure_reason = str(reason)
        target.miss_count = 0

        # Geometrically false targets and uncertain physical picks must not be
        # sent back to the robot.  Collision-risk failures are retryable only
        # for edge targets, where the second attempt uses a different staged
        # approach rather than repeating the same route.
        if reason in (
            'SUCTION_TOO_FAR',
            'VISION_INVALID',
            'PICK_NOT_REMOVED',
        ):
            target.state = 'BLOCKED'
            return

        retry_limit = (
            self.edge_max_retries
            if self.is_edge_target(target)
            else self.max_retries
        )

        if reason == 'COLLISION_RISK' and not self.is_edge_target(target):
            target.state = 'BLOCKED'
            return

        if target.attempts <= retry_limit:
            # Do not immediately retry the same coordinates.  Deferred
            # planning failures are considered only after all other pending
            # objects have had a chance to run.
            target.state = 'DEFERRED'
        else:
            target.state = (
                'BLOCKED'
                if self.is_edge_target(target)
                else 'FAILED'
            )

    def reactivate_deferred(self):
        """Release deferred targets after the rest of the queue is drained."""
        changed = 0
        for record in self.records:
            if record.state == 'DEFERRED':
                record.state = 'PENDING'
                changed += 1
        return changed

    def _find(self, target_id):
        return next(
            (record for record in self.records
             if record.target_id == target_id),
            None,
        )

    def _greedy_matches(self, records, positions):
        pairs = []
        for record_index, record in enumerate(records):
            for position_index, position in enumerate(positions):
                pairs.append((
                    self._distance(record.position, position),
                    record_index,
                    position_index,
                ))

        matched_records = set()
        matched_positions = set()
        matches = []
        for distance, record_index, position_index in sorted(pairs):
            if distance > self.match_distance:
                break
            if record_index in matched_records or position_index in matched_positions:
                continue
            matched_records.add(record_index)
            matched_positions.add(position_index)
            matches.append((record_index, position_index))

        return matches, matched_records, matched_positions

    def reconcile(self, positions, excluded_ids=None):
        """
        Associate a new detector snapshot with queued scene records.

        Returns `(added, removed)` target records.  Records that disappear are
        marked MISSING first and only removed after repeated confirmations.
        """
        excluded_ids = set(excluded_ids or ())
        positions = [tuple(position) for position in positions]
        records = [
            record for record in self.records
            if record.state in (
                'PENDING',
                'MISSING',
                'DEFERRED',
                'BLOCKED',
                'FAILED',
            )
            and record.target_id not in excluded_ids
        ]

        matches, matched_records, matched_positions = self._greedy_matches(
            records,
            positions,
        )

        for record_index, position_index in matches:
            record = records[record_index]
            record.position = positions[position_index]
            if record.state == 'MISSING':
                record.state = 'PENDING'
            record.miss_count = 0
            record.last_seen = time.monotonic()

        removed = []
        for record_index, record in enumerate(records):
            if record_index in matched_records:
                continue
            record.miss_count += 1
            previous_state = record.state
            record.state = 'MISSING'
            if record.miss_count >= self.missing_confirmations:
                self.records.remove(record)
                removed.append(record)
            elif previous_state in ('BLOCKED', 'FAILED', 'DEFERRED'):
                # Preserve scheduling policy while tolerating short visual
                # occlusions.  The record can still be removed after enough
                # consecutive misses.
                record.state = previous_state

        added = []
        for position_index, position in enumerate(positions):
            if position_index in matched_positions:
                continue
            added.append(self._append(position))

        self._sort()
        return added, removed

    def finish_attempt(
        self,
        target_id,
        positions,
        success,
        failure_reason='PLANNING_FAILED',
    ):
        """Commit or recover one reserved attempt after a new scene snapshot."""
        target = self._find(target_id)
        if target is None:
            return False

        positions = [tuple(position) for position in positions]
        nearest_index = None
        nearest_distance = None
        for index, position in enumerate(positions):
            distance = self._distance(target.position, position)
            if nearest_distance is None or distance < nearest_distance:
                nearest_index = index
                nearest_distance = distance

        target_still_visible = (
            nearest_distance is not None
            and nearest_distance <= self.match_distance
        )

        if success:
            # picker.run() reports success only after an attached object has
            # completed the full transfer and has been released in the
            # receiving bin.  A detection near the old source coordinate can
            # therefore be a previously occluded or stacked cube.  Remove the
            # completed reservation first and let reconciliation create or
            # match every object that remains in the source scene.
            self.records.remove(target)
            remaining_positions = positions
            self.reconcile(remaining_positions)
            return True

        # A failed action must not silently delete the record.
        if target_still_visible:
            target.position = positions[nearest_index]
            positions = [
                position for index, position in enumerate(positions)
                if index != nearest_index
            ]
        target.miss_count = 0
        self.fail(target.target_id, failure_reason)
        self.reconcile(positions, excluded_ids={target.target_id})
        return False

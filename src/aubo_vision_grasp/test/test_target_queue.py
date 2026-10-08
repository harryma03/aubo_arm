import pytest

from aubo_vision_grasp.target_queue import TargetQueue


def test_newly_exposed_target_is_added_after_reconcile():
    queue = TargetQueue(match_distance=0.05)
    queue.seed([(0.0, -0.7, -0.75)])

    added, removed = queue.reconcile([
        (0.01, -0.7, -0.75),
        (0.20, -0.7, -0.75),
    ])

    assert len(added) == 1
    assert not removed
    assert len(queue.records) == 2


def test_successful_pick_requeues_newly_exposed_nearby_target():
    queue = TargetQueue(match_distance=0.120)
    queue.seed([(-0.139, -0.868, -0.726)])
    selected = queue.reserve_next()

    removed = queue.finish_attempt(
        selected.target_id,
        [(-0.175, -0.885, -0.726)],
        success=True,
    )

    assert removed is True
    assert len(queue.records) == 1
    assert queue.records[0].target_id != selected.target_id
    assert queue.records[0].state == 'PENDING'
    assert queue.records[0].position == (-0.175, -0.885, -0.726)


def test_no_fresh_scene_holds_reserved_target():
    queue = TargetQueue()
    queue.seed([(0.0, -0.7, -0.75)])
    target = queue.reserve_next()

    queue.hold_for_confirmation(target.target_id)

    assert target.state == 'WAITING_CONFIRMATION'
    assert queue.reserve_next() is None


def test_reserve_prefers_isolated_target_over_nearest_target():
    queue = TargetQueue()
    queue.seed([
        (0.00, -0.40, -0.75),
        (0.02, -0.48, -0.75),
        (0.20, -0.90, -0.75),
    ])

    target = queue.reserve_next()

    assert target.position == (0.20, -0.90, -0.75)


def test_reserve_prefers_higher_target_before_clearance():
    queue = TargetQueue()
    queue.seed([
        (0.00, -0.70, -0.75),
        (0.02, -0.72, -0.65),
        (0.20, -0.90, -0.75),
    ])

    target = queue.reserve_next()

    assert target.position == (0.02, -0.72, -0.65)


def test_edge_grasp_moves_toward_hopper_center_on_both_axes():
    queue = TargetQueue(
        hopper_center=(0.0, -0.75),
        safe_half=(0.2275, 0.4100),
    )
    queue.seed([(0.2200, -0.3500, -0.75)])
    target = queue.records[0]

    grasp, reason = queue.compensated_grasp_position(
        target,
        trigger=0.080,
        offset=0.010,
    )

    assert grasp == pytest.approx((0.2100, -0.3600, -0.75))
    assert reason == 'INWARD_XY'


def test_edge_grasp_keeps_center_when_another_target_would_be_nearest():
    queue = TargetQueue(
        hopper_center=(0.0, -0.75),
        safe_half=(0.2275, 0.4100),
    )
    queue.seed([
        (0.2200, -0.3500, -0.75),
        (0.2140, -0.3560, -0.75),
    ])
    target = queue.records[0]

    grasp, reason = queue.compensated_grasp_position(
        target,
        trigger=0.080,
        offset=0.010,
    )

    assert grasp == target.position
    assert reason == 'CENTER_NEAREST_TARGET_GUARD'


def test_edge_collision_gets_one_deferred_adaptive_retry():
    queue = TargetQueue(
        edge_margin=0.080,
        edge_max_retries=1,
        hopper_center=(0.0, -0.75),
        safe_half=(0.2275, 0.4100),
    )
    queue.seed([(0.2200, -0.75, -0.75)])

    first = queue.reserve_next()
    queue.fail(first.target_id, 'COLLISION_RISK')
    assert first.state == 'DEFERRED'

    assert queue.reactivate_deferred() == 1
    second = queue.reserve_next()
    assert second.attempts == 2

    queue.fail(second.target_id, 'COLLISION_RISK')
    assert second.state == 'BLOCKED'


def test_non_edge_collision_is_not_retried():
    queue = TargetQueue(
        edge_margin=0.080,
        edge_max_retries=1,
        hopper_center=(0.0, -0.75),
        safe_half=(0.2275, 0.4100),
    )
    queue.seed([(0.0, -0.75, -0.75)])

    target = queue.reserve_next()
    queue.fail(target.target_id, 'COLLISION_RISK')

    assert target.state == 'BLOCKED'

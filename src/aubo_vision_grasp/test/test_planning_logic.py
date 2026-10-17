"""Planning policy regressions without a running robot or ROS graph."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from aubo_vision_grasp import visual_pick_to_bin as picking
from aubo_vision_grasp import visual_pick_queue as scheduling


def candidate_picker(budget=100.0):
    return SimpleNamespace(
        system_config=SimpleNamespace(motion=SimpleNamespace(
            grasp_search_budget=budget, grasp_candidate_count=3)),
        GRASP_YAW_CANDIDATES=(0.0, 0.5, 1.0),
        _plan_approach=lambda label, x, y, z, yaw: (10 - yaw,),
        robot_state_at_trajectory_end=lambda trajectory: trajectory,
        _plan_descent=lambda *args: (1, 'CARTESIAN'),
        _state_with_lifted_payload=lambda state, *args, **kwargs: state,
        _plan_pick_exit=lambda *args: (1, 'CARTESIAN'),
        compute_cartesian_waypoints=lambda *args, **kwargs: 1,
        _trajectory_joint_travel=lambda *trajectories: sum(trajectories),
        get_logger=lambda: Mock(),
        exit_corridor_x=0.0, exit_corridor_y=0.0, pick_clear_z=0.2,
        place_x=-0.6, place_y=-0.8, transfer_z=0.2,
    )


def test_candidate_search_selects_shorter_later_chain():
    picker = candidate_picker()
    picking.VisualPickToBin._preflight_pick_candidates(picker, 0, 0, 0.1, 0)
    assert picker.grasp_yaw == 1.0


def test_zero_budget_preserves_first_feasible_policy():
    picker = candidate_picker(budget=0)
    picking.VisualPickToBin._preflight_pick_candidates(picker, 0, 0, 0.1, 0)
    assert picker.grasp_yaw == 0.0


def test_cartesian_candidate_beats_shorter_ompl_descent():
    picker = candidate_picker()
    picker._plan_descent = lambda *args: (
        (1, 'CARTESIAN') if args[-1] == 0.0 else (0, 'OMPL_FALLBACK')
    )
    picking.VisualPickToBin._preflight_pick_candidates(picker, 0, 0, 0.1, 0)
    assert picker.grasp_yaw == 0.0


def test_partial_second_retreat_requires_replanning_from_current_state():
    picker = SimpleNamespace(
        _place_retreat_mode='SEGMENTED_CARTESIAN',
        _place_retreat_segments=(1, 2),
        _at_place_release=True, _at_place_entry=False,
        execute_trajectory=Mock(side_effect=[None, RuntimeError('partial motion')]),
        _phase='PLACE_RETREAT', _attached=False,
        recovery_timeout=12, _active_place_tcp=(-0.6, -0.8),
        place_entry_z=-0.05, grasp_yaw=0,
        ompl_move_to=Mock(), get_logger=lambda: Mock(),
    )
    with pytest.raises(RuntimeError, match='partial motion'):
        picking.VisualPickToBin._execute_place_retreat(picker, -0.6, -0.8)
    assert not picker._at_place_entry
    assert not picker._at_place_release
    picking.VisualPickToBin.recover_after_failure(picker, 0, 0, 0.1, 0)
    picker.ompl_move_to.assert_called_once()
    assert picker._phase == 'RECOVERED_PLACE_ENTRY'


def test_failed_retreat_recovery_is_not_reported_as_success():
    picker = SimpleNamespace(
        _phase='PLACE_RETREAT', _attached=False,
        _at_place_entry=False, _at_place_release=False,
        _place_retreat_started=True, _active_place_tcp=(-0.6, -0.8),
        recovery_timeout=12, place_entry_z=-0.05, grasp_yaw=0,
        ompl_move_to=Mock(side_effect=RuntimeError('planning failed')),
    )
    with pytest.raises(RuntimeError, match='PLACE_ENTRY recovery failed'):
        picking.VisualPickToBin.recover_after_failure(picker, 0, 0, 0.1, 0)
    assert picker._phase == 'PLACE_RETREAT'


def test_moving_ambiguous_obstacle_resets_snapshot_stability(monkeypatch):
    clock = [0.0]
    monitor = SimpleNamespace(
        latest_positions=[(0, 0, 0)], last_update=0,
        update_sequence=0, latest_ambiguous_positions=[],
        source_evidence_update=0, source_evidence_count=1,
        stable_frames=2, filter_ambiguous_targets=lambda points: points,
        _max_position_delta=scheduling.TargetArrayMonitor._max_position_delta,
    )

    def next_frame(*args, **kwargs):
        clock[0] += 1
        monitor.update_sequence += 1
        monitor.last_update = clock[0]
        # Same count, but the obstacle moves in each of the first 3 frames.
        monitor.latest_ambiguous_positions = [(min(clock[0], 3), 0, 0)]

    monkeypatch.setattr(scheduling.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(scheduling.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(scheduling.rclpy, 'spin_once', next_frame)
    result = scheduling.TargetArrayMonitor.wait_for_stable_snapshot(
        monitor, stable_time=1, timeout=10)
    assert result == [(0, 0, 0)]
    assert monitor.update_sequence == 4

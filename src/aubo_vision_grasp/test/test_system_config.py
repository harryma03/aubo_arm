import pytest

from aubo_vision_grasp.system_config import load_system_config


def test_workcell_geometry_is_derived_from_one_snapshot(monkeypatch):
    monkeypatch.setenv('AUBO_WORKCELL_X', '0.10')
    monkeypatch.setenv('AUBO_WORKCELL_Y', '0.20')
    monkeypatch.setenv('AUBO_HOPPER_LOCAL_X', '0.30')
    monkeypatch.setenv('AUBO_HOPPER_LOCAL_Y', '-0.40')
    monkeypatch.setenv('AUBO_HOPPER_BOTTOM_SIZE_X', '0.60')
    monkeypatch.setenv('AUBO_CUBE_SIZE_X', '0.10')
    monkeypatch.setenv('AUBO_HOPPER_WALL_MARGIN', '0.02')

    workcell = load_system_config().workcell

    assert workcell.hopper_x == pytest.approx(0.40)
    assert workcell.hopper_y == pytest.approx(-0.20)
    assert workcell.safe_half_x == pytest.approx(0.23)


def test_workcell_loads_cube_drop_height(monkeypatch):
    monkeypatch.setenv('AUBO_CUBE_DROP_HEIGHT', '0.04')

    assert load_system_config().workcell.cube_drop_height == pytest.approx(0.04)


def test_vision_processing_rate_has_stable_default(monkeypatch):
    monkeypatch.delenv('AUBO_VISION_PROCESSING_RATE', raising=False)

    assert load_system_config().vision.processing_rate == 5.0


def test_initial_scene_wait_has_a_short_bounded_default(monkeypatch):
    monkeypatch.delenv('AUBO_INITIAL_SCENE_SETTLE_TIME', raising=False)

    assert load_system_config().queue.initial_scene_settle_time == 2.0


def test_edge_grasp_compensation_defaults_are_conservative(monkeypatch):
    names = (
        'AUBO_TARGET_EDGE_MAX_RETRIES',
        'AUBO_EDGE_GRASP_COMPENSATION',
        'AUBO_EDGE_GRASP_TRIGGER',
        'AUBO_EDGE_GRASP_OFFSET',
    )
    for name in names:
        monkeypatch.delenv(name, raising=False)

    queue = load_system_config().queue

    assert queue.edge_grasp_compensation is True
    assert queue.edge_grasp_trigger == pytest.approx(0.080)
    assert queue.edge_grasp_offset == pytest.approx(0.020)
    assert queue.edge_max_retries == 1


def test_vision_display_is_enabled_by_default(monkeypatch):
    monkeypatch.delenv('AUBO_VISION_DISPLAY', raising=False)

    assert load_system_config().vision.display_enabled is True


def test_fixed_ready_pose_requires_six_joint_values(monkeypatch):
    monkeypatch.setenv(
        'AUBO_READY_JOINTS',
        '-1.310177,-2.005036,-1.938566,-1.506058,1.598025,1.715036',
    )

    assert load_system_config().motion.ready_joint_values == pytest.approx(
        (-1.310177, -2.005036, -1.938566, -1.506058, 1.598025, 1.715036)
    )

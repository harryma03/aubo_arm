import pytest

from aubo_vision_grasp.system_config import load_system_config
from aubo_vision_grasp.visual_pick_queue import receiving_bin_place_slots


def test_receiving_bin_has_six_unique_slots(monkeypatch):
    monkeypatch.setenv('AUBO_WORKCELL_X', '0.0')
    monkeypatch.setenv('AUBO_WORKCELL_Y', '0.0')

    slots = receiving_bin_place_slots(load_system_config().workcell)

    assert len(slots) == 6
    assert len(set(slots)) == 6


def test_receiving_bin_slots_follow_workcell_offset(monkeypatch):
    monkeypatch.setenv('AUBO_WORKCELL_X', '0.2')
    monkeypatch.setenv('AUBO_WORKCELL_Y', '-0.1')

    slots = receiving_bin_place_slots(load_system_config().workcell)

    assert slots[0] == pytest.approx((-0.455, -0.84))


def test_last_receiving_bin_row_is_shifted_inward(monkeypatch):
    monkeypatch.setenv('AUBO_WORKCELL_X', '0.0')
    monkeypatch.setenv('AUBO_WORKCELL_Y', '0.0')

    slots = receiving_bin_place_slots(load_system_config().workcell)

    assert slots[-2] == pytest.approx((-0.655, -1.020))
    assert slots[-1] == pytest.approx((-0.495, -1.020))

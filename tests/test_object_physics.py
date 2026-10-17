"""Check passive object physics without starting or modifying Gazebo."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/aubo_workcell_description/models"


def test_cube_inertia_matches_mass_and_geometry():
    model = ET.parse(ROOT / "models/grasp_cube/model.sdf").getroot().find("model")
    assert model.findtext("static") == "false"
    link = model.find("link")
    assert link.findtext("gravity") == "true"
    mass = float(link.findtext("inertial/mass"))
    x, y, z = map(float, link.findtext("collision/geometry/box/size").split())
    expected = [(y*y + z*z), (x*x + z*z), (x*x + y*y)]
    for axis, value in zip(("ixx", "iyy", "izz"), expected):
        assert float(link.findtext(f"inertial/inertia/{axis}")) == pytest.approx(
            mass * value / 12, rel=1e-8,
        )
    assert 0 < float(link.findtext("velocity_decay/linear")) <= 0.005
    assert 0 < float(link.findtext("velocity_decay/angular")) <= 0.01


def test_floor_friction_matches_cube_without_sticky_hopper_walls():
    cube = ET.parse(ROOT / "models/grasp_cube/model.sdf").getroot()
    scene = ET.parse(SCENE / "workcell_environment/model.sdf").getroot()
    ground = ET.parse(SCENE / "workcell_ground/model.sdf").getroot()
    contacts = [
        cube.find(".//collision"),
        scene.find(".//collision[@name='hopper_bottom_collision']"),
        scene.find(".//link[@name='store']/collision"),
        ground.find(".//collision"),
    ]
    for collision in contacts:
        for axis in ("mu", "mu2"):
            assert float(collision.findtext(f"surface/friction/ode/{axis}")) == 1.5
        for axis in ("slip1", "slip2"):
            assert float(collision.findtext(f"surface/friction/ode/{axis}")) == 0
        assert float(collision.findtext(
            "surface/bounce/restitution_coefficient", default="0",
        )) == 0
    assert scene.findtext("model/static") == "true"
    for side in ("left", "right", "front", "back"):
        wall = scene.find(f".//collision[@name='hopper_{side}_collision']")
        assert float(wall.findtext("surface/friction/ode/mu")) == 0.8


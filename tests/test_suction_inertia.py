"""Regression checks for the preserved Gazebo suction fixed-joint chain."""
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = (
    ROOT / "src/aubo_ros2_driver/aubo_description/urdf/xacro/inc"
)


def inertia_for_link(path, name):
    root = ET.parse(path).getroot()
    link = root.find(f".//link[@name='{name}']")
    assert link is not None
    mass = float(link.find("inertial/mass").attrib["value"])
    inertia = link.find("inertial/inertia").attrib
    return mass, tuple(float(inertia[axis]) for axis in ("ixx", "iyy", "izz"))


def test_preserved_end_links_do_not_use_near_zero_mass():
    ee_mass, ee_inertia = inertia_for_link(
        DESCRIPTION / "aubo_ros2_macro.xacro", "ee_link",
    )
    tcp_mass, tcp_inertia = inertia_for_link(
        DESCRIPTION / "aubo_ros2.xacro", "suction_tcp",
    )

    assert ee_mass >= 0.01
    assert tcp_mass >= 0.01
    assert min(ee_inertia) >= 1e-7
    assert min(tcp_inertia) >= 1e-7

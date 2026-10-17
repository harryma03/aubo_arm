#!/usr/bin/env python3

import math
import os
import random
import subprocess
import time
from pathlib import Path

N = 4
WORKCELL_X = float(os.environ.get("AUBO_WORKCELL_X", "0.0"))
WORKCELL_Y = float(os.environ.get("AUBO_WORKCELL_Y", "0.0"))
WORKCELL_Z = float(os.environ.get("AUBO_WORKCELL_Z", "-0.95"))
HOPPER_X = WORKCELL_X + float(
    os.environ.get("AUBO_HOPPER_LOCAL_X", "0.00000335")
)
HOPPER_Y = WORKCELL_Y + float(
    os.environ.get("AUBO_HOPPER_LOCAL_Y", "-0.75005309")
)
# Keep this legacy generator aligned with workcell_layout.env.
CUBE_Z = float(os.environ.get("AUBO_CUBE_DROP_HEIGHT", "0.040")) + (
    WORKCELL_Z - (-0.95)
)
CUBE_MODEL_FILE = Path(
    os.environ.get(
        "CUBE_MODEL_FILE",
        str(Path(__file__).resolve().parent / "models" / "grasp_cube" / "model.sdf")
    )
)

if not CUBE_MODEL_FILE.is_file():
    raise FileNotFoundError(
        f"Cube model not found: {CUBE_MODEL_FILE}. "
        "Set CUBE_MODEL_FILE to a Gazebo SDF/URDF file."
    )

# 先使用比较保守、已经验证可抓的区域
X_MIN = -0.20
X_MAX =  0.20

# Bounds relative to the current hopper center. At the default pose this is
# the historical world range [-0.98, -0.60] m.
Y_MIN = -0.23
Y_MAX =  0.15

# 5 cm cube，保持足够间距，避免互相碰撞/遮挡
MIN_DIST = 0.14

points = []

for _ in range(N):
    found = False

    for _try in range(1000):
        x = HOPPER_X + random.uniform(X_MIN, X_MAX)
        y = HOPPER_Y + random.uniform(Y_MIN, Y_MAX)

        ok = True

        for px, py in points:
            d = math.hypot(x - px, y - py)

            if d < MIN_DIST:
                ok = False
                break

        if ok:
            points.append((x, y))
            found = True
            break

    if not found:
        raise RuntimeError("Cannot generate non-overlapping cube positions")

print("")
print("=" * 55)
print("RANDOM 4 CUBES")
print("=" * 55)

# 删除旧方块
for i in range(1, N + 1):

    subprocess.run(
        [
            "ros2",
            "service",
            "call",
            "/delete_entity",
            "gazebo_msgs/srv/DeleteEntity",
            f"{{name: 'grasp_cube_{i}'}}"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

time.sleep(0.5)

for i, (x, y) in enumerate(points, start=1):

    name = f"grasp_cube_{i}"

    print(
        f"{name}: "
        f"x={x:+.4f}, "
        f"y={y:+.4f}"
    )

    subprocess.run(
        [
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity",
            name,
            "-file",
            str(CUBE_MODEL_FILE),
            "-x",
            f"{x:.4f}",
            "-y",
            f"{y:.4f}",
            "-z",
            f"{CUBE_Z:.4f}"
        ],
        check=True
    )

    time.sleep(0.3)

print("=" * 55)
print("4 cubes spawned.")
print("=" * 55)

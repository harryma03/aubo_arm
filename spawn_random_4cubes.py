#!/usr/bin/env python3

import math
import random
import subprocess
import time

N = 4

# 先使用比较保守、已经验证可抓的区域
X_MIN = -0.20
X_MAX =  0.20

Y_MIN = -0.98
Y_MAX = -0.60

# 5 cm cube，保持足够间距，避免互相碰撞/遮挡
MIN_DIST = 0.14

points = []

for _ in range(N):
    found = False

    for _try in range(1000):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

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
            "/tmp/grasp_cube.sdf",
            "-x",
            f"{x:.4f}",
            "-y",
            f"{y:.4f}",
            "-z",
            "0.08"
        ],
        check=True
    )

    time.sleep(0.3)

print("=" * 55)
print("4 cubes spawned.")
print("=" * 55)

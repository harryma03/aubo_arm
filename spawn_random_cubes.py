#!/usr/bin/env python3

import random
import subprocess
import time


N = 4

# 12 个安全候选位置，每次随机选 4 个
XS = [-0.16, -0.08, 0.00, 0.08]
YS = [-0.95, -0.86, -0.70]

cells = [(x, y) for y in YS for x in XS]
random.shuffle(cells)
selected = cells[:N]


def run_quiet(cmd, timeout=4):
    try:
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False
        )
    except subprocess.TimeoutExpired:
        pass


print()
print("=" * 55)
print("RESET OLD CUBES")
print("=" * 55)

# ------------------------------------------------------------
# 1. 确保吸盘关闭
# ------------------------------------------------------------

print("[1] suction_off")

run_quiet([
    "ros2",
    "service",
    "call",
    "/suction_off",
    "std_srvs/srv/Trigger",
    "{}"
])


# ------------------------------------------------------------
# 2. 只删除当前使用的 4 个 cube
# ------------------------------------------------------------

for i in range(1, 5):

    name = f"grasp_cube_{i}"

    print(f"[2] delete {name}")

    run_quiet([
        "ros2",
        "service",
        "call",
        "/delete_entity",
        "gazebo_msgs/srv/DeleteEntity",
        f"{{name: '{name}'}}"
    ])


# 删除旧的单物体测试对象
print("[2] delete grasp_cube")

run_quiet([
    "ros2",
    "service",
    "call",
    "/delete_entity",
    "gazebo_msgs/srv/DeleteEntity",
    "{name: 'grasp_cube'}"
])

time.sleep(0.5)


# ------------------------------------------------------------
# 3. 随机生成 4 个
# ------------------------------------------------------------

print()
print("=" * 55)
print("SPAWN 4 RANDOM CUBES")
print("=" * 55)

for i, (base_x, base_y) in enumerate(selected, start=1):

    # 小范围随机扰动，因此每轮位置都不同
    x = base_x + random.uniform(-0.012, 0.012)
    y = base_y + random.uniform(-0.012, 0.012)

    name = f"grasp_cube_{i}"

    print(
        f"{name}: "
        f"x={x:+.4f}, "
        f"y={y:+.4f}"
    )

    result = subprocess.run([
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
    ])

    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to spawn {name}, "
            f"returncode={result.returncode}"
        )

    time.sleep(0.25)


print()
print("=" * 55)
print("SPAWN COMPLETE: 4 cubes")
print("=" * 55)
